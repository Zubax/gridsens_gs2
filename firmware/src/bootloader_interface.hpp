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

#pragma once

#include <cstdint>
#include <uavcan/uavcan.hpp>

namespace bootloader_interface
{
/**
 * Reads the bootloader's shared data structure.
 * This function must be invoked before the CAN hardware is initialized.
 */
void init();

/**
 * Returns version info of the currently running firmware image.
 * This function relies on the firmware image post-processing; refer to the build system docs for details.
 */
uavcan::protocol::SoftwareVersion makeUavcanSoftwareVersionStruct();

/**
 * If known, returns the bit rate value inherited from the bootloader.
 * If unknown, return zero.
 */
std::uint32_t getInheritedCanBusBitRate();

/**
 * If known, returns the node ID value inherited from the bootloader.
 * If unknown, returns an invalid node ID.
 */
uavcan::NodeID getInheritedNodeID();

/**
 * Initializes the shared data structure with given values.
 */
void passParametersToBootloader(std::uint32_t can_bus_bit_rate, uavcan::NodeID node_id);

}
