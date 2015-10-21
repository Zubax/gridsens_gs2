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

#include "bootloader_interface.hpp"
#include "board/board.hpp"
#include "node.hpp"
#include "air_sensor.hpp"
#include "magnetometer.hpp"
#include "gnss.hpp"
#include "cli/cli.hpp"

namespace
{

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
    bootloader_interface::init();
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
