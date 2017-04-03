/*
 * Copyright (C) 2014-2015  Zubax Robotics  <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cassert>
#include <utility>
#include <zubax_chibios/os.hpp>
#include <bootloader_interface/bootloader_interface.hpp>
#include "board/board.hpp"
#include "node.hpp"
#include "air_sensor.hpp"
#include "magnetometer.hpp"
#include "gnss.hpp"
#include "usb/usb.hpp"
#include "nmea/nmea.hpp"

namespace
{

os::config::Param<bool> nmea_uart_on("nmea.uart_on", false);

/**
 * This callback is invoked from the local UAVCAN node (from its own thread!) when the said node
 * receives a firmware update request. The objective here is to set up the bootloader and signal
 * rebooting; once the system is rebooted, the bootloader will know what to do.
 */
auto onFirmwareUpdateRequestedFromUAVCAN(
    const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& request)
{
    /*
     * Checking preconditions
     */
    static bool already_in_progress = false;

    const std::uint8_t source_node_id =
        ((request.source_node_id > 0) &&
         (request.source_node_id <= uavcan::NodeID::Max) &&
         uavcan::NodeID(request.source_node_id).isUnicast()) ?
            request.source_node_id :
            request.getSrcNodeID().get();

    os::lowsyslog("UAVCAN firmware update request from %d, source %d, path '%s'\n",
                  request.getSrcNodeID().get(),
                  source_node_id,
                  request.image_file_remote_path.path.c_str());

    if (already_in_progress)
    {
        os::lowsyslog("UAVCAN firmware update is already in progress, rejecting\n");
        return uavcan::protocol::file::BeginFirmwareUpdate::Response::ERROR_IN_PROGRESS;
    }

    /*
     * Initializing the app shared structure with proper arguments
     */
    node::Lock locker;

    bootloader_interface::AppShared shared;
    shared.can_bus_speed = node::getCANBitRate();
    shared.uavcan_node_id = node::getNode().getNodeID().get();
    shared.uavcan_fw_server_node_id = source_node_id;
    shared.stay_in_bootloader = true;

    std::strncpy(static_cast<char*>(&shared.uavcan_file_name[0]),       // This is really messy
                 request.image_file_remote_path.path.c_str(),
                 shared.UAVCANFileNameMaxLength);
    shared.uavcan_file_name[shared.UAVCANFileNameMaxLength - 1] = '\0';

    static_assert(request.image_file_remote_path.path.MaxSize < shared.UAVCANFileNameMaxLength, "Err...");

    os::lowsyslog("Bootloader args: CAN bus bitrate: %u, local node ID: %d\n",
                  unsigned(shared.can_bus_speed), shared.uavcan_node_id);

    /*
     * Commiting everything
     */
    bootloader_interface::writeSharedStruct(shared);

    os::requestReboot();

    already_in_progress = true;

    os::lowsyslog("UAVCAN firmware update initiated\n");
    return uavcan::protocol::file::BeginFirmwareUpdate::Response::ERROR_OK;
}

std::pair<unsigned, unsigned> getStatusLedOnOffMSecDurations()
{
    using uavcan::protocol::NodeStatus;

    const auto health = node::getWorstComponentHealth();
    if (health == NodeStatus::HEALTH_OK)      { return {50, 950}; }
    if (health == NodeStatus::HEALTH_WARNING) { return {50, 250}; }
    return {50, 50};  // ERROR/CRITICAL
}

}

int main()
{
    /*
     * Component initialization; threads start here.
     */
    auto wdt = board::init(1100);

    {
        const auto fw_version = bootloader_interface::getFirmwareVersion();

        const auto app_shared_read_result = bootloader_interface::readAndInvalidateSharedStruct();
        const auto& app_shared = app_shared_read_result.first;
        const auto app_shared_available = app_shared_read_result.second;

        if (app_shared_available)
        {
            os::lowsyslog("Bootloader struct values: CAN bitrate: %u, UAVCAN Node ID: %u\n",
                             unsigned(app_shared.can_bus_speed), app_shared.uavcan_node_id);
        }
        else
        {
            os::lowsyslog("Bootloader struct is NOT present\n");
        }

        node::init(app_shared_available ? app_shared.can_bus_speed : 0,
                   app_shared_available ? app_shared.uavcan_node_id : 0,
                   {fw_version.major, fw_version.minor},
                   fw_version.image_crc64we,
                   fw_version.vcs_commit,
                   &onFirmwareUpdateRequestedFromUAVCAN);
    }

    air_sensor::init();
    gnss::init();
    magnetometer::init();
    nmea::init();

    usb::init();

    /*
     * Higher-level initialization.
     */
    chibios_rt::BaseThread::setPriority(LOWPRIO);

    if (nmea_uart_on)
    {
        nmea::addOutput(reinterpret_cast<::BaseChannel*>(&STDOUT_SD));
    }

    /*
     * Main loop.
     */
    auto config_modifications = os::config::getModificationCounter();
    bool save_config_on_next_iteration = false;

    while (!os::isRebootRequested())
    {
        const auto on_off = getStatusLedOnOffMSecDurations();
        board::setStatusLed(true);
        ::usleep(on_off.first * 1000);
        wdt.reset();
        board::setStatusLed(false);
        ::usleep(on_off.second * 1000);
        wdt.reset();

        // Saving config if it was flagged on the previous iteration and no new modifications happened since then
        if (save_config_on_next_iteration)
        {
            board::setStatusLed(true);                                  // Extra indication

            save_config_on_next_iteration = false;

            if (os::config::getModificationCounter() == config_modifications)
            {
                os::lowsyslog("Saving configuration... ");
                const int res = ::configSave();                         // TODO use C++ API
                os::lowsyslog("Config saved, status %d\n", res);
            }
            else
            {
                os::lowsyslog("Config save postponed\n");               // There were new modifications, deferring
            }
        }
        else
        {
            // If config was changed, scheduling it to save on the next iteration
            const auto new_config_modifications = os::config::getModificationCounter();
            if (new_config_modifications != config_modifications)
            {
                config_modifications = new_config_modifications;
                save_config_on_next_iteration = true;
                os::lowsyslog("Config save scheduled\n");
            }
        }
    }

    board::setStatusLed(true);
    ::usleep(500000);                   // Let threads terminate properly
    board::restart();                   // Then reset anyway

    return 0;
}


#define MATCH_GCC_VERSION(major, minor, patch)  \
    ((__GNUC__ == (major)) && (__GNUC_MINOR__ == (minor)) && (__GNUC_PATCHLEVEL__ == (patch)))

#if !(MATCH_GCC_VERSION(6, 3, 1) || MATCH_GCC_VERSION(5, 4, 1))
# error "This compiler is not supported"
#endif
