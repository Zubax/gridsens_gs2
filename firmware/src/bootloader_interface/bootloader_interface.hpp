/*
 * Copyright (C) 2014-2017  Zubax Robotics  <info@zubax.com>
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
#include <utility>


namespace bootloader_interface
{
/**
 * This struct is used to exchange data between the bootloader and the application.
 * Its format allows for future extensions.
 * Note that the binary layout of this structure must not be changed, as it will break the bootloader.
 */
struct AppShared
{
    std::uint32_t reserved_a = 0;                               ///< Reserved for future use
    std::uint32_t reserved_b = 0;                               ///< Reserved for future use

    /*
     * UAVCAN part
     */
    std::uint32_t can_bus_speed = 0;                            ///< App <-> Bootloader
    std::uint8_t uavcan_node_id = 0;                            ///< App <-> Bootloader
    std::uint8_t uavcan_fw_server_node_id = 0;                  ///< App --> Bootloader

    static constexpr std::uint8_t UAVCANFileNameMaxLength = 201;
    char uavcan_file_name[UAVCANFileNameMaxLength] = {};        ///< App --> Bootloader

    /*
     * General part
     */
    bool stay_in_bootloader = false;                            ///< App --> Bootloader
};

static_assert(sizeof(AppShared) <= 240, "AppShared may be larger than the amount of allocated memory");
static_assert(sizeof(bool) == 1, "Please redefine bool as uint8_t in the shared struct (should never happen on ARM)");

/**
 * Descriptor of the firmware that is currently being executed.
 * This descriptor is used (at least) by the UAVCAN node and CLI interface.
 */
struct FirmwareVersion
{
    std::uint64_t image_crc64we = 0;    ///< CRC-64-WE, same as in libuavcan
    std::uint32_t vcs_commit = 0;       ///< 32-bit commit hash of the firmware source
    std::uint8_t major = 0;             ///< Major version number
    std::uint8_t minor = 0;             ///< Minor version number
};

/**
 * Returns version info of the currently running firmware image.
 * This function relies on the firmware image post-processing; refer to the build system docs for details.
 */
FirmwareVersion getFirmwareVersion();

/**
 * Reads the bootloader-app shared structure from the shared memory location and returns it by value.
 * The operation will fail if the structure is not written correctly (e.g. if the bootloader didn't provide any
 * information or if it was using wrong structure layout due to version mismatch, or whatever).
 * The success of the operation is indicated in the second member of the returned pair.
 * Note that the structure will be invalidated after read to prevent deja-vu.
 */
std::pair<AppShared, bool> readAndInvalidateSharedStruct();

/**
 * Writes the bootloader-app shared data structure.
 * This function cannot fail.
 */
void writeSharedStruct(const AppShared& shared);

}
