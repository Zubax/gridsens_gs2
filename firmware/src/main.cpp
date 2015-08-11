/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cassert>
#include <utility>

#include <zubax_chibios/sys/sys.h>
#include <zubax_chibios/watchdog/watchdog.hpp>

#include "board/board.hpp"
#include "node.hpp"
#include "air_sensor.hpp"
#include "magnetometer.hpp"
#include "gnss.hpp"
#include "cli.hpp"

namespace
{
/**
 * This is the Brickproof Bootloader's app descriptor.
 * Details: https://github.com/PX4/Firmware/tree/nuttx_next/src/drivers/bootloaders/src/uavcan
 */
const volatile class __attribute__((packed))
{
    std::uint8_t signature[8] = {'A','P','D','e','s','c','0','0'};
    std::uint64_t image_crc = 0;
    std::uint32_t image_size = 0;
    std::uint32_t vcs_commit = GIT_HASH;
    std::uint8_t major_version = FW_VERSION_MAJOR;
    std::uint8_t minor_version = FW_VERSION_MINOR;
    std::uint8_t reserved[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
} _app_descriptor __attribute__((section(".app_descriptor")));

std::pair<unsigned, unsigned> getStatusLedOnOffMSecDurations()
{
    using uavcan::protocol::NodeStatus;

    if (node::getNode().getNodeStatusProvider().getMode() == NodeStatus::MODE_OPERATIONAL)
    {
        const auto health = node::getNode().getNodeStatusProvider().getHealth();
        if (health == NodeStatus::HEALTH_OK)      { return {100, 900}; }
        if (health == NodeStatus::HEALTH_WARNING) { return {100, 200}; }
        return {100, 100};  // ERROR/CRITICAL
    }
    else
    {
        return {500, 500};
    }
}

}

int main()
{
    board::init();
    node::init();
    air_sensor::init();
    gnss::init();
    magnetometer::init();

    zubax_chibios::watchdog::Timer wdt;
    wdt.startMSec(1100);

    cli::init();

    chibios_rt::BaseThread::setPriority(LOWPRIO);

    while (!node::hasPendingRestartRequest())
    {
        const auto on_off = getStatusLedOnOffMSecDurations();
        board::setStatusLed(true);
        ::usleep(on_off.first * 1000);
        wdt.reset();
        board::setStatusLed(false);
        ::usleep(on_off.second * 1000);
        wdt.reset();
    }

    board::setStatusLed(true);
    ::usleep(500000);                   // Let threads terminate properly
    board::restart();                   // Then reset anyway

    return 0;
}
