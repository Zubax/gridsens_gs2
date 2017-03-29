/*
 * Copyright (C) 2017  Zubax Robotics  <info@zubax.com>
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

#pragma once

#include <cstdint>
#include <array>
#include <zubax_chibios/os.hpp>

namespace board
{

os::watchdog::Timer init(unsigned wdt_timeout_ms);

void setCANLed(unsigned iface_index, bool state);
void setStatusLed(bool state);

__attribute__((noreturn))
void restart();

__attribute__((noreturn))
void bootApplication();

typedef std::array<std::uint8_t, 12> UniqueID;
UniqueID readUniqueID();

typedef std::array<std::uint8_t, 128> DeviceSignature;
bool tryReadDeviceSignature(DeviceSignature& out_sign);

struct HardwareVersion
{
    std::uint8_t major;
    std::uint8_t minor;
};

HardwareVersion detectHardwareVersion();

inline unsigned getFlashSize()
{
    return 1024 * *reinterpret_cast<std::uint16_t*>(0x1FFFF7E0);
}

}
