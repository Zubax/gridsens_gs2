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

#include "bootloader_interface.hpp"
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
    bootloader_interface::init();
    auto wdt = board::init(1100);
    node::init();
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
