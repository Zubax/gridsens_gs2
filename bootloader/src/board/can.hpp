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

#include <canard_stm32.h>
#include <zubax_chibios/bootloader/loaders/uavcan.hpp>
#include <board/board.hpp>
#include <hal.h>


namespace board
{
namespace can
{
/**
 * An adapter class that bridges the Libcanard STM32 driver with the UAVCAN bootloader interface driver.
 * Note that the logic of the return codes implemented in the Libcanard STM32 drivers matches the expectations
 * of the bootloader, so on this part no adaptation is needed.
 */
class CANIfaceAdapter : public os::bootloader::uavcan_loader::ICANIface
{
    /**
     * Hardware FIFO is 3 frames deep; the shortest (worst case) frame duration with acceptance filters allowing only
     * extended frames is:
     *  (1+11+1+1+18+1+2+4+0+15+1+1+1+7) = 64 bits = 64 microseconds @ 1 Mbps
     *
     * Multiply by the depth of the FIFO:
     *  64 * 3 = 192 microseconds
     *
     * Therefore, we must read the driver not less frequently than every 192 microseconds, otherwise we might be
     * losing frames due to RX overrun. Therefore we enforce that the system tick interval is less than that.
     */
    static_assert((1000000 / CH_CFG_ST_FREQUENCY) < 180,
                  "Minimal delay must be lower in order for the libcanard STM32 driver to work properly");

    static constexpr unsigned LEDUpdateIntervalMilliseconds = 25;

    bool had_activity_ = false;
    ::systime_t last_led_update_timestamp_st_ = 0;

    inline void wait()
    {
        if (chVTTimeElapsedSinceX(last_led_update_timestamp_st_) >= MS2ST(LEDUpdateIntervalMilliseconds))
        {
            last_led_update_timestamp_st_ += MS2ST(LEDUpdateIntervalMilliseconds);
            board::setCANLed(0, had_activity_);
            had_activity_ = false;
        }

        chThdSleep(1);      // Sleeping for a minimal possible time interval
    }

public:
    int init(const std::uint32_t bitrate,
             const Mode mode,
             const AcceptanceFilterConfig& acceptance_filter) override
    {
        DEBUG_LOG("CAN init: %9u bps, mode %d, filt id 0x%x mask 0x%x\n",
                  unsigned(bitrate),
                  int(mode),
                  unsigned(acceptance_filter.id),
                  unsigned(acceptance_filter.mask));

        had_activity_ = false;
        last_led_update_timestamp_st_ = chVTGetSystemTimeX();

        // Computing CAN timings
        CanardSTM32CANTimings timings{};
        int res = canardSTM32ComputeCANTimings(STM32_PCLK1, bitrate, &timings);
        if (res < 0)
        {
            return res;
        }

        // Initializing the interface
        CanardSTM32IfaceMode mode2{};
        switch (mode)
        {
        case Mode::Normal:
        {
            mode2 = CanardSTM32IfaceModeNormal;
            break;
        }
        case Mode::Silent:
        {
            mode2 = CanardSTM32IfaceModeSilent;
            break;
        }
        case Mode::AutomaticTxAbortOnError:
        {
            mode2 = CanardSTM32IfaceModeAutomaticTxAbortOnError;
            break;
        }
        }

        res = canardSTM32Init(&timings, mode2);
        if (res < 0)
        {
            return res;
        }

        // Configuring acceptance filters
        CanardSTM32AcceptanceFilterConfiguration acceptance_filter_config_array[1];
        acceptance_filter_config_array[0].id   = acceptance_filter.id;
        acceptance_filter_config_array[0].mask = acceptance_filter.mask;

        res = canardSTM32ConfigureAcceptanceFilters(&acceptance_filter_config_array[0], 1);
        if (res < 0)
        {
            return res;
        }

        return 0;
    }

    int send(const CanardCANFrame& frame, const int timeout_millisec) override
    {
        const auto started_at = chVTGetSystemTimeX();
        do
        {
            int res = canardSTM32Transmit(&frame);      // Try to transmit
            if (res != 0)
            {
                had_activity_ |= res > 0;
                return res;                             // Either success or error, return
            }
            wait();                                     // No space in the buffer, skip the time quantum and try again
        }
        while (int(ST2MS(chVTTimeElapsedSinceX(started_at))) < timeout_millisec);

        return 0;                                       // Timed out
    }

    std::pair<int, CanardCANFrame> receive(const int timeout_millisec) override
    {
        const auto started_at = chVTGetSystemTimeX();
        CanardCANFrame f{};
        do
        {
            int res = canardSTM32Receive(&f);
            if (res != 0)
            {
                had_activity_ |= res > 0;
                return {res, f};                        // Either success or error, return
            }
            wait();                                     // Buffer is empty, skip the time quantum and try again
        }
        while (int(ST2MS(chVTTimeElapsedSinceX(started_at))) < timeout_millisec);

        return {0, f};                                  // Timed out
    }
};

}
}
