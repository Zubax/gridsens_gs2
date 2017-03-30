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

#include "uavcan.hpp"
#include <zubax_chibios/bootloader/loaders/uavcan.hpp>
#include <board/can.hpp>


namespace uavcan
{

static os::bootloader::uavcan_loader::UAVCANFirmwareUpdateNode<> g_node;

static board::can::CANIfaceAdapter g_iface;


void init(os::bootloader::Bootloader& bl,
          const std::uint32_t can_bit_rate,
          const std::uint8_t local_node_id,
          const std::uint8_t file_server_node_id,
          const char* const remote_image_file_path)
{
    (void) g_node.start(NORMALPRIO + 10,
                        bl,
                        g_iface,
                        can_bit_rate,
                        local_node_id,
                        file_server_node_id,
                        remote_image_file_path);
}


Parameters getParameters()
{
    Parameters p;
    p.can_bus_bit_rate = g_node.getCANBusBitRate();
    p.local_node_id    = g_node.getLocalNodeID();
    return p;
}

}
