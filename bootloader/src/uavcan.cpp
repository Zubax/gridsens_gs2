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
#include <zubax_chibios/util/helpers.hpp>
#include <board/can.hpp>
#include <board/board.hpp>


namespace uavcan
{

static os::helpers::LazyConstructor<os::bootloader::uavcan_loader::UAVCANFirmwareUpdateNode<>> g_node;

static board::can::CANIfaceAdapter g_iface;


void init(os::bootloader::Bootloader& bl,
          const std::uint32_t can_bit_rate,
          const std::uint8_t local_node_id,
          const std::uint8_t file_server_node_id,
          const char* const remote_image_file_path)
{
    const auto uid_short = board::readUniqueID();

    os::bootloader::uavcan_loader::NodeUniqueID uid_full{};
    std::fill(uid_full.begin(), uid_full.end(), 0);
    std::copy(uid_short.begin(),
              uid_short.end(),
              uid_full.begin());

    g_node.construct<os::bootloader::Bootloader&,
                     os::bootloader::uavcan_loader::ICANIface&,
                     const os::bootloader::uavcan_loader::NodeUniqueID&>(bl, g_iface, uid_full);

    (void) g_node->start(NORMALPRIO + 10,
                         can_bit_rate,
                         local_node_id,
                         file_server_node_id,
                         remote_image_file_path);
}


Parameters getParameters()
{
    Parameters p;
    if (g_node.isConstructed())
    {
        p.can_bus_bit_rate = g_node->getCANBusBitRate();
        p.local_node_id    = g_node->getLocalNodeID();
    }
    return p;
}

}
