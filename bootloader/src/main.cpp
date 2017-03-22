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

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cstdlib>
#include <cassert>
#include <utility>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/platform/stm32/flash_writer.hpp>
#include <zubax_chibios/bootloader/bootloader.hpp>
#include "board/board.hpp"
#include "board/usb_cdc_acm.hpp"
#include "cli.hpp"


namespace app
{
namespace
{
/**
 * This watchdog timeout will be applied to the bootloader itself, and also to the application boot process.
 * In other words, the application will have to reset the watchdog in this time after boot.
 */
constexpr unsigned WatchdogTimeoutMSec = 5000;

/**
 * This class contains logic and hardcoded values that are SPECIFIC FOR THIS PARTICULAR MCU AND APPLICATION.
 */
class AppStorageBackend : public bootloader::IAppStorageBackend
{
    static constexpr unsigned ApplicationAddress = FLASH_BASE + APPLICATION_OFFSET;

    static unsigned getFlashSize()
    {
        return 1024 * *reinterpret_cast<std::uint16_t*>(0x1FFFF7E0);
    }

    static bool correctOffsetAndSize(std::size_t& offset, std::size_t& size)
    {
        const auto flash_end = FLASH_BASE + getFlashSize();
        offset += ApplicationAddress;
        if (offset >= flash_end)
        {
            return false;
        }
        if ((offset + size) >= flash_end)
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
        DEBUG_LOG("Flash size: %u Bytes\n", getFlashSize());
    }

    int beginUpgrade() override
    {
        const unsigned erase_address = ApplicationAddress;
        const unsigned erase_size = getFlashSize() - APPLICATION_OFFSET;

        DEBUG_LOG("Flash erase: address 0x%x, %u Bytes\n", erase_address, erase_size);

        os::stm32::FlashWriter writer;
        const bool ok = writer.erase(reinterpret_cast<void*>(erase_address), erase_size);

        DEBUG_LOG("Flash erase result: %s\n", ok ? "OK" : "FAILURE");

        return ok ? 0 : -ErrEraseFailed;
    }

    int endUpgrade(bool) override { return 0; }

    int write(std::size_t offset, const void* data, std::size_t size) override
    {
        if (!correctOffsetAndSize(offset, size))
        {
            return 0;
        }

        os::stm32::FlashWriter writer;
        return writer.write(reinterpret_cast<const void*>(offset), data, size) ? size : -ErrWriteFailed;
    }

    int read(std::size_t offset, void* data, std::size_t size) override
    {
        if (!correctOffsetAndSize(offset, size))
        {
            return 0;
        }
        std::memmove(data, reinterpret_cast<const void*>(offset), size);
        return size;
    }
};


static inline std::pair<unsigned, unsigned> bootloaderStateToLEDOnOffDurationMSec(bootloader::State state)
{
    switch (state)
    {
    case bootloader::State::NoAppToBoot:
    {
        return {50, 50};
    }
    case bootloader::State::BootCancelled:
    {
        return {50, 950};
    }
    case bootloader::State::AppUpgradeInProgress:
    {
        return {500, 500};
    }
    case bootloader::State::BootDelay:
    case bootloader::State::ReadyToBoot:
    {
        return {0, 0};
    }
    }
    assert(false);
    return {0, 0};
}

}
}


int main()
{
    /*
     * Initialization
     */
    auto wdt = board::init(app::WatchdogTimeoutMSec);

    wdt.reset();
    board::usb_cdc_acm::init();
    wdt.reset();

    chibios_rt::BaseThread::setPriority(LOWPRIO);

    /*
     * Bootloader logic initialization
     */
    app::AppStorageBackend backend;

    bootloader::Bootloader bl(backend);

    cli::init(bl);

    /*
     * Main loop
     */
    while (!os::isRebootRequested())
    {
        wdt.reset();

        const auto bl_state = bl.getState();

        board::setStatusLed(true);      // Always on
        const auto duration = app::bootloaderStateToLEDOnOffDurationMSec(bl_state);
        if (duration.first == 0 && duration.second == 0)
        {
            board::setCANLed(0, false);
            board::setCANLed(1, false);
            chThdSleepMilliseconds(100);
        }
        else
        {
            board::setCANLed(0, true);
            board::setCANLed(1, true);
            chThdSleepMilliseconds(duration.first);
            board::setCANLed(0, false);
            board::setCANLed(1, false);
            chThdSleepMilliseconds(duration.second);
        }
    }

    return 0;
}
