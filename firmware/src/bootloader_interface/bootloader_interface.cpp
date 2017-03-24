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

#include "bootloader_interface.hpp"
#include <zubax_chibios/bootloader/app_shared.hpp>


namespace bootloader_interface
{
/**
 * This is the Brickproof Bootloader's app descriptor.
 * Details: https://github.com/PX4/Firmware/tree/nuttx_next/src/drivers/bootloaders/src/uavcan
 */
static const volatile struct __attribute__((packed))
{
    std::uint8_t signature[8]   = {'A','P','D','e','s','c','0','0'};
    std::uint64_t image_crc     = 0;
    std::uint32_t image_size    = 0;
    std::uint32_t vcs_commit    = GIT_HASH;
    std::uint8_t major_version  = FW_VERSION_MAJOR;
    std::uint8_t minor_version  = FW_VERSION_MINOR;
    std::uint8_t reserved[6]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
} _app_descriptor __attribute__((section(".app_descriptor")));

static chibios_rt::Mutex g_mutex;


FirmwareVersion getFirmwareVersion()
{
    FirmwareVersion x;
    x.image_crc64we = _app_descriptor.image_crc;
    x.vcs_commit = _app_descriptor.vcs_commit;
    x.major = _app_descriptor.major_version;
    x.minor = _app_descriptor.minor_version;
    return x;
}

static inline auto makeMarshaller()
{
    // Note that the first 256 bytes of SRAM are used for bootloader-app communication! See the linker script.
    return os::bootloader::app_shared::makeAppSharedMarshaller<AppShared>(reinterpret_cast<void*>(SRAM_BASE));
}

std::pair<AppShared, bool> readAndInvalidateSharedStruct()
{
    os::MutexLocker locker(g_mutex);
    return makeMarshaller().read(os::bootloader::app_shared::AutoErase::EraseAfterRead);
}

void writeSharedStruct(const AppShared& shared)
{
    os::MutexLocker locker(g_mutex);
    makeMarshaller().write(shared);
}

}
