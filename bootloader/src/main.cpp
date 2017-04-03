/*
 * Copyright (C) 2014-2017  Zubax Robotics  <info@zubax.com>
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
#include <cstdlib>
#include <cassert>
#include <utility>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/platform/stm32/flash_writer.hpp>
#include <zubax_chibios/bootloader/bootloader.hpp>
#include "bootloader_app_interface.hpp"
#include "board/board.hpp"
#include "board/app_storage_backend.hpp"
#include "board/usb_cdc_acm.hpp"
#include "uavcan.hpp"
#include "cli.hpp"


namespace app
{
namespace
{
/**
 * This watchdog timeout will be applied to the bootloader itself, and also to the application boot process.
 * In other words, the application will have to reset the watchdog in this time after boot.
 */
constexpr unsigned WatchdogTimeoutMSec = 5000;


static inline std::pair<unsigned, unsigned> bootloaderStateToLEDOnOffDurationMSec(os::bootloader::State state)
{
    switch (state)
    {
    case os::bootloader::State::NoAppToBoot:
    {
        return {50, 50};
    }
    case os::bootloader::State::BootCancelled:
    {
        return {50, 950};
    }
    case os::bootloader::State::AppUpgradeInProgress:
    {
        return {500, 500};
    }
    case os::bootloader::State::BootDelay:
    case os::bootloader::State::ReadyToBoot:
    {
        return {0, 0};
    }
    }
    assert(false);
    return {0, 0};
}

}
}


int main()
{
    /*
     * Initialization
     */
    auto wdt = board::init(app::WatchdogTimeoutMSec);

    wdt.reset();
    board::usb_cdc_acm::init();
    wdt.reset();

    chibios_rt::BaseThread::setPriority(LOWPRIO);

    /*
     * Bootloader logic initialization
     */
    board::AppStorageBackend backend;

    const ::systime_t bootloader_init_started_at = chVTGetSystemTimeX();

    // This class is large, do not allocate it on the stack
    static os::bootloader::Bootloader bl(backend, board::getFlashSize());

    os::lowsyslog("Bootloader: Inited in %u ms\n", unsigned(ST2MS(chVTTimeElapsedSinceX(bootloader_init_started_at))));

    cli::init(bl);

    /*
     * Parsing the app shared struct and initializing the UAVCAN component accordingly
     */
    {
        std::pair<bootloader_app_interface::AppShared, bool> app_shared = bootloader_app_interface::readAndErase();

        if (app_shared.second)
        {
            if (app_shared.first.stay_in_bootloader)
            {
                bl.cancelBoot();
            }

            // Ensuring the string is properly terminated:
            app_shared.first.uavcan_file_name[app_shared.first.UAVCANFileNameMaxLength - 1] = '\0';

            os::lowsyslog("AppShared: Wait %c; UAVCAN: %u bps %u/%u \"%s\"\n",
                          app_shared.first.stay_in_bootloader ? 'Y' : 'N',
                          unsigned(app_shared.first.can_bus_speed),
                          app_shared.first.uavcan_node_id,
                          app_shared.first.uavcan_fw_server_node_id,
                          &app_shared.first.uavcan_file_name[0]);

            uavcan::init(bl,
                         app_shared.first.can_bus_speed,
                         app_shared.first.uavcan_node_id,
                         app_shared.first.uavcan_fw_server_node_id,
                         &app_shared.first.uavcan_file_name[0]);
        }
        else
        {
            uavcan::init(bl);
        }
    }

    /*
     * Main loop
     */
    while (!os::isRebootRequested())
    {
        wdt.reset();

        const auto bl_state = bl.getState();
        if (bl_state == os::bootloader::State::ReadyToBoot)
        {
            break;
        }

        board::setStatusLed(true);      // Always on
        const auto duration = app::bootloaderStateToLEDOnOffDurationMSec(bl_state);
        if (duration.first == 0 && duration.second == 0)
        {
            chThdSleepMilliseconds(100);
        }
        else
        {
            board::setCANLed(1, true);                  // Using CAN2 LED for state indication purposes
            chThdSleepMilliseconds(duration.first);
            board::setCANLed(1, false);
            chThdSleepMilliseconds(duration.second);
        }
    }

    if (os::isRebootRequested())
    {
        os::lowsyslog("\nREBOOTING\n");
        chThdSleepMilliseconds(500);       // Providing some time for other components to react
        board::restart();
    }

    /*
     * Starting the application
     */
    os::lowsyslog("\nBOOTING\n");
    os::requestReboot();         // Notifying other components that we're going down
    chThdSleepMilliseconds(500); // Providing some time for other components to react

    // Writing down the CAN bus parameters for the application, if known
    {
        const uavcan::Parameters p = uavcan::getParameters();

        bootloader_app_interface::AppShared as;
        as.can_bus_speed  = p.can_bus_bit_rate;
        as.uavcan_node_id = p.local_node_id;
        assert((as.can_bus_speed  <= 1000000) &&
               (as.uavcan_node_id <= 127));

        bootloader_app_interface::write(as);
    }

    wdt.reset();                 // The final reset, the application will have time to boot and init until next timeout
    board::bootApplication();

    return 0;
}


#define MATCH_GCC_VERSION(major, minor, patch)  \
    ((__GNUC__ == (major)) && (__GNUC_MINOR__ == (minor)) && (__GNUC_PATCHLEVEL__ == (patch)))

#if !(MATCH_GCC_VERSION(6, 3, 1) || MATCH_GCC_VERSION(5, 4, 1))
# error "This compiler is not supported"
#endif
