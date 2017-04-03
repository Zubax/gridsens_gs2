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

#include "board.hpp"
#include <cstring>
#include <cstdlib>


namespace board
{
/**
 * This class contains logic and hardcoded values that are SPECIFIC FOR THIS PARTICULAR MCU AND APPLICATION.
 */
class AppStorageBackend : public os::bootloader::IAppStorageBackend
{
    static constexpr unsigned ApplicationAddress = FLASH_BASE + APPLICATION_OFFSET;

    static bool correctOffsetAndSize(std::size_t& offset, std::size_t& size)
    {
        const auto flash_end = FLASH_BASE + board::getFlashSize();
        offset += ApplicationAddress;
        if UNLIKELY(offset >= flash_end)
        {
            return false;
        }
        if UNLIKELY((offset + size) >= flash_end)
        {
            size = flash_end - offset;
        }
        return true;
    }

public:
    static constexpr std::int16_t ErrEraseFailed = 9001;
    static constexpr std::int16_t ErrWriteFailed = 9002;

    AppStorageBackend()
    {
        DEBUG_LOG("Flash %u B\n", board::getFlashSize());
    }

    int beginUpgrade()   override { return 0; }
    int endUpgrade(bool) override { return 0; }

    int write(std::size_t offset, const void* data, std::size_t size) override
    {
        if (!correctOffsetAndSize(offset, size))
        {
            return 0;
        }

        os::stm32::FlashWriter writer;

        const bool ok = writer.erase(reinterpret_cast<void*>(offset), size);
        if (!ok)
        {
            return -ErrEraseFailed;
        }

        return writer.write(reinterpret_cast<const void*>(offset), data, size) ? size : -ErrWriteFailed;
    }

    int read(std::size_t offset, void* data, std::size_t size) const override
    {
        if LIKELY(correctOffsetAndSize(offset, size))
        {
            std::memcpy(data, reinterpret_cast<const void*>(offset), size);
            return size;
        }
        else
        {
            return 0;
        }
    }
};

}
