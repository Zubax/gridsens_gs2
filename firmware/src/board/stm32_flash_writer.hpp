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

#include <ch.hpp>
#include <hal.h>
#include <cassert>
#include <cstring>
#include <cstdint>

namespace stm32_flash_writer
{
/**
 * The code below assumes that HSI oscillator is up and running,
 * otherwise the Flash controller (FPEC) may misbehave.
 * Any FPEC issues will be detected at run time during write/erase verification.
 */
class Writer
{
    static void waitReady()
    {
        do
        {
            assert(!(FLASH->SR & FLASH_SR_PGERR));
            assert(!(FLASH->SR & FLASH_SR_WRPRTERR));
        }
        while (FLASH->SR & FLASH_SR_BSY);
        FLASH->SR |= FLASH_SR_EOP;
    }

    struct Prologuer
    {
        Prologuer()
        {
            chSysLock();
            waitReady();
            if (FLASH->CR & FLASH_CR_LOCK)
            {
                FLASH->KEYR = 0x45670123UL;
                FLASH->KEYR = 0xCDEF89ABUL;
            }
            FLASH->SR |= FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR; // Reset flags
            FLASH->CR = 0;
        }

        ~Prologuer()
        {
            FLASH->CR = FLASH_CR_LOCK;  // Reset the FPEC configuration and lock
            chSysUnlock();
        }
    };

public:
    /**
     * Source and destination must be aligned at two bytes.
     */
    bool write(const void* const where,
               const void* const what,
               const unsigned how_much)
    {
        if (((reinterpret_cast<std::size_t>(where)) % 2 != 0) ||
            ((reinterpret_cast<std::size_t>(what)) % 2 != 0) ||
            (where == nullptr) || (what == nullptr))
        {
            assert(false);
            return false;
        }

        const unsigned num_halfwords = (how_much + 1U) / 2U;

        volatile std::uint16_t* flashptr16 = static_cast<std::uint16_t*>(const_cast<void*>(where));
        const std::uint16_t* ramptr16 = static_cast<const std::uint16_t*>(what);

        Prologuer prologuer;

        FLASH->CR = FLASH_CR_PG;

        for (unsigned i = 0; i < num_halfwords; i++)
        {
            *flashptr16++ = *ramptr16++;
            waitReady();
        }

        waitReady();
        FLASH->CR = 0;

        return std::memcmp(what, where, how_much) == 0;
    }
};

}
