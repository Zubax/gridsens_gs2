/*
 * Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
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

#include <hal.h>
#include <zubax_chibios/bootloader/app_shared.hpp>


namespace bootloader_app_interface
{
/**
 * This struct is used to exchange data between the bootloader and the application.
 * Its format allows for future extensions.
 */
struct AppShared
{
    std::uint32_t reserved_a = 0;                               ///< Reserved for future use
    std::uint32_t reserved_b = 0;                               ///< Reserved for future use

    /*
     * UAVCAN part
     */
    std::uint32_t can_bus_speed = 0;
    std::uint8_t uavcan_node_id = 0;
    std::uint8_t uavcan_fw_server_node_id = 0;

    static constexpr std::uint8_t UAVCANFileNameMaxLength = 201;
    char uavcan_file_name[UAVCANFileNameMaxLength] = {};

    /*
     * General part
     */
    bool stay_in_bootloader = false;
};


static_assert(sizeof(AppShared) <= 240, "AppShared may be larger than the amount of allocated memory");


static inline auto makeMarshaller()
{
    // Note that the first 256 bytes of SRAM are used for bootloader-app communication! See the linker script.
    return os::bootloader::app_shared::makeAppSharedMarshaller<AppShared>(reinterpret_cast<void*>(SRAM_BASE));
}


static inline auto readAndErase()
{
    return makeMarshaller().read(os::bootloader::app_shared::AutoErase::EraseAfterRead);
}

static inline void write(const AppShared& apsh)
{
    makeMarshaller().write(apsh);
}

}
