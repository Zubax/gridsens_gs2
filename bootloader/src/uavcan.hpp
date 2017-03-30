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

#include <zubax_chibios/bootloader/bootloader.hpp>
#include <zubax_chibios/util/heapless.hpp>
#include <cstdint>


namespace uavcan
{
/**
 * Starts the firmware update node. All parameters except the bootloader reference are optional.
 *
 * @param bl                        reference to the bootloader instance
 * @param can_bit_rate              bit rate of the CAN bus, if supplied by the application, otherwise 0
 * @param local_node_id             local node ID, if supplied by the application, otherwise 0
 * @param file_server_node_id       node ID of the remote node that will provide the file, if known, otherwise 0
 * @param remote_image_file_path    path of the firmware image file on the remote file server, if known, otherwise null
 */
void init(os::bootloader::Bootloader& bl,
          const std::uint32_t can_bit_rate = 0,
          const std::uint8_t local_node_id = 0,
          const std::uint8_t file_server_node_id = 0,
          const char* const remote_image_file_path = nullptr);

/**
 * Runtime estimated UAVCAN bus parameters.
 * Unknown parameters are set to zero.
 */
struct Parameters
{
    std::uint32_t can_bus_bit_rate = 0;         ///< Set if CAN bus bit rate auto detection was successful
    std::uint8_t local_node_id = 0;             ///< Set if dynamic node ID allocation was successful
};

Parameters getParameters();

}
