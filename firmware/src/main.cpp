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

std::pair<unsigned, unsigned> getStatusLedOnOffMSecDurations()
{
    const auto self_status = node::getNode().getNodeStatusProvider().getStatusCode();
    using uavcan::protocol::NodeStatus;
    if (self_status == NodeStatus::STATUS_INITIALIZING) { return {500, 500}; }
    if (self_status == NodeStatus::STATUS_OK)           { return {100, 900}; }
    if (self_status == NodeStatus::STATUS_WARNING)      { return {100, 300}; }
    return {100, 100};  // CRITICAL and all
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

    while (1)
    {
        const auto on_off = getStatusLedOnOffMSecDurations();
        board::setStatusLed(true);
        ::usleep(on_off.first * 1000);
        wdt.reset();
        board::setStatusLed(false);
        ::usleep(on_off.second * 1000);
        wdt.reset();
    }
    return 0;
}
