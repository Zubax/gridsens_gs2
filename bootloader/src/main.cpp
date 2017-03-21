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
#include "board/board.hpp"

namespace
{



}

int main()
{
    auto wdt = board::init(1100);

    while (!os::isRebootRequested())
    {
        wdt.reset();

        ::usleep(30000);
        board::setStatusLed(false);
        board::setCANLed(0, true);

        ::usleep(30000);
        board::setCANLed(0, false);
        board::setCANLed(1, true);

        ::usleep(30000);
        board::setCANLed(1, false);
        board::setStatusLed(true);
    }

    return 0;
}
