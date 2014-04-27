/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cassert>

#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/config/config.hpp>
#include <crdr_chibios/watchdog/watchdog.hpp>

#include "board/board.hpp"
#include "node.hpp"
#include "air_sensor.hpp"
#include "magnetometer.hpp"
#include "gnss.hpp"

namespace
{

std::pair<unsigned, unsigned> getStatusLedOnOffMSecDurations()
{
    const auto self_status = node::getNode().getNodeStatusProvider().getStatusCode();
    using uavcan::protocol::NodeStatus;
    if (self_status == NodeStatus::STATUS_INITIALIZING) { return {500, 500}; }
    if (self_status == NodeStatus::STATUS_OK)           { return {100, 900}; }
    if (self_status == NodeStatus::STATUS_WARNING)      { return {100, 200}; }
    return {100, 100};  // CRITICAL and all
}

void init()
{
    board::init();
    node::init();
    air_sensor::init();
    gnss::init();
    magnetometer::init();
}

}

int main()
{
    init();

    while (1)
    {
        const auto on_off = getStatusLedOnOffMSecDurations();
        board::setStatusLed(true);
        usleep(on_off.first * 1000);
        board::setStatusLed(false);
        usleep(on_off.second * 1000);
    }
    return 0;
}
