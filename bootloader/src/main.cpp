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
#include <canard_stm32.h>
#include "bootloader_app_interface.hpp"
#include "board/board.hpp"
#include "board/app_storage_backend.hpp"
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


static inline std::pair<unsigned, unsigned> bootloaderStateToLEDOnOffDurationMSec(os::bootloader::State state)
{
    switch (state)
    {
    case os::bootloader::State::NoAppToBoot:
    {
        return {50, 50};
    }
    case os::bootloader::State::BootCancelled:
    {
        return {50, 950};
    }
    case os::bootloader::State::AppUpgradeInProgress:
    {
        return {500, 500};
    }
    case os::bootloader::State::BootDelay:
    case os::bootloader::State::ReadyToBoot:
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

    {
        int res = 0;

        CanardSTM32CANTimings timings;
        res = canardSTM32ComputeCANTimings(STM32_PCLK1, 1000000, &timings);
        if (res < 0)
        {
            chibios_rt::System::halt("CAN timings");
        }

        DEBUG_LOG("CAN %u %u/%u\n", timings.bit_rate_prescaler, timings.bit_segment_1, timings.bit_segment_2);

        res = ::canardSTM32Init(&timings, ::CanardSTM32IfaceModeAutomaticTxAbortOnError);
        if (res < 0)
        {
            chibios_rt::System::halt("CAN init");
        }

        ::CanardSTM32AcceptanceFilterConfiguration filters[3];

        filters[0].id   = 0x0001557F | CANARD_CAN_FRAME_EFF;
        filters[0].mask = 0x00FFFFFF;

        filters[1].id   = 0x003FF27E | CANARD_CAN_FRAME_EFF;
        filters[1].mask = 0x00FFFFFF;

        filters[2].id   = 0x123;
        filters[2].mask = 0x7FF | CANARD_CAN_FRAME_EFF;

        res = ::canardSTM32ConfigureAcceptanceFilters(filters, 3);
        if (res < 0)
        {
            chibios_rt::System::halt("CAN filts");
        }
    }

    chibios_rt::BaseThread::setPriority(LOWPRIO);

    /*
     * Bootloader logic initialization
     */
    board::AppStorageBackend backend;

    os::bootloader::Bootloader bl(backend);

    cli::init(bl);

    /*
     * Parsing the app shared struct
     */
    const std::pair<bootloader_app_interface::AppShared, bool> app_shared = bootloader_app_interface::readAndErase();
    if (app_shared.second)
    {
        os::lowsyslog("AppShared: CAN %u bps   NID %u/%u   Wait %c\n",
                      unsigned(app_shared.first.can_bus_speed),
                      app_shared.first.uavcan_node_id,
                      app_shared.first.uavcan_fw_server_node_id,
                      app_shared.first.stay_in_bootloader ? 'Y' : 'N');

        if (app_shared.first.stay_in_bootloader)
        {
            bl.cancelBoot();
        }
    }

    /*
     * Main loop
     */
    while (!os::isRebootRequested())
    {
        wdt.reset();

        const auto bl_state = bl.getState();
        if (bl_state == os::bootloader::State::ReadyToBoot)
        {
            break;
        }

        board::setStatusLed(true);      // Always on
        const auto duration = app::bootloaderStateToLEDOnOffDurationMSec(bl_state);
        if (duration.first == 0 && duration.second == 0)
        {
            chThdSleepMilliseconds(100);
        }
        else
        {
            board::setCANLed(1, true);                  // Using CAN2 LED for state indication purposes
            chThdSleepMilliseconds(duration.first);
            board::setCANLed(1, false);
            chThdSleepMilliseconds(duration.second);
        }

        // XXX Libcanard test
        while (true)
        {
            CanardCANFrame frame;
            int res = ::canardSTM32Receive(&frame);
            if (res < 0)
            {
                chibios_rt::System::halt("CAN RX");
            }
            if (res == 0)
            {
                break;
            }
            const auto stats = ::canardSTM32GetStats();
            os::lowsyslog("RX %x %u [ovf %u err %u]\n",
                          unsigned(frame.id),
                          frame.data_len,
                          unsigned(stats.rx_overflow_count),
                          unsigned(stats.error_count));

            res = ::canardSTM32Transmit(&frame);
            if (res < 0)
            {
                chibios_rt::System::halt("CAN TX");
            }
            if (res == 0)
            {
                os::lowsyslog("CAN TX SKIPPED\n");
            }
        }

        CanardCANFrame frame;
        frame.data_len = 1;
        frame.id = 0x123 | CANARD_CAN_FRAME_EFF;
        int res = ::canardSTM32Transmit(&frame);
        const auto stats = ::canardSTM32GetStats();
        os::lowsyslog("CAN TX %d [ovf %u err %u]\n",
                      res,
                      unsigned(stats.rx_overflow_count),
                      unsigned(stats.error_count));
    }

    if (os::isRebootRequested())
    {
        os::lowsyslog("\nREBOOTING\n");
        chThdSleepMilliseconds(500);       // Providing some time for other components to react
        board::restart();
    }

    /*
     * Starting the application
     */
    os::lowsyslog("\nBOOTING\n");
    os::requestReboot();         // Notifying other components that we're going down
    chThdSleepMilliseconds(500); // Providing some time for other components to react
    wdt.reset();                 // The final reset, the application will have time to boot and init until next timeout
    board::bootApplication();

    return 0;
}


#define MATCH_GCC_VERSION(major, minor, patch)  \
    ((__GNUC__ == (major)) && (__GNUC_MINOR__ == (minor)) && (__GNUC_PATCHLEVEL__ == (patch)))

#if !MATCH_GCC_VERSION(5, 4, 1)
# error "This compiler is not supported"
#endif
