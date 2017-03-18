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

#include <ch.hpp>
#include <hal.h>
#include <cstdint>
#include <utility>
#include <array>
#include <cassert>
#include <unistd.h>
#include <zubax_chibios/os.hpp>

/**
 * Measurement Specialities MS5611 sensor driver
 */
namespace ms5611
{

constexpr float DegreesCelsiusToKelvinOffset = 273.15F;

/**
 * One measurement.
 */
struct Sample
{
    float pressure = 0;         ///< Pascal
    float temperature = 0;      ///< Kelvin
};

/**
 * MS5611 driver logic.
 */
class MS5611
{
    struct Command
    {
        static constexpr std::uint8_t Reset           = 0x1E;   ///< Reset command for MS5611

        // D1 CONVERSION
        static constexpr std::uint8_t ConvD1_OSR256   = 0x40;   ///< Convert D1 (oversampling rate = 256)
        static constexpr std::uint8_t ConvD1_OSR512   = 0x42;   ///< Convert D1 (oversampling rate = 512)
        static constexpr std::uint8_t ConvD1_OSR1024  = 0x44;   ///< Convert D1 (oversampling rate = 1024)
        static constexpr std::uint8_t ConvD1_OSR2048  = 0x46;   ///< Convert D1 (oversampling rate = 2048)
        static constexpr std::uint8_t ConvD1_OSR4096  = 0x48;   ///< Convert D1 (oversampling rate = 4096)

        // D2 CONVERSION
        static constexpr std::uint8_t ConvD2_OSR256   = 0x50;   ///< Convert D2 (oversampling rate = 256)
        static constexpr std::uint8_t ConvD2_OSR512   = 0x52;   ///< Convert D2 (oversampling rate = 512)
        static constexpr std::uint8_t ConvD2_OSR1024  = 0x54;   ///< Convert D2 (oversampling rate = 1024)
        static constexpr std::uint8_t ConvD2_OSR2048  = 0x56;   ///< Convert D2 (oversampling rate = 2048)
        static constexpr std::uint8_t ConvD2_OSR4096  = 0x58;   ///< Convert D2 (oversampling rate = 4096)

        static constexpr std::uint8_t ADCRead         = 0x00;   ///< ADC READ
        static constexpr std::uint8_t PromBase        = 0xA0;   ///< PROM BASE ADRRESS 0b1010xxx0
    };

    union PROM
    {
        struct
        {
            std::uint16_t resv;                         ///< Reserved
            std::uint16_t c1_pressure_sens;             ///< Pressure sensitivity coef.
            std::uint16_t c2_pressure_offset;           ///< Pressure offset coef.
            std::uint16_t c3_temp_coeff_pres_sens;      ///< Temperature coefficient of pressure sensitivity
            std::uint16_t c4_temp_coeff_pres_offset;    ///< Temperature coefficient coefficient of pressure offset
            std::uint16_t c5_reference_temp;            ///< Reference temperature coef.
            std::uint16_t c6_temp_coeff_temp;           ///< Temperature coefficient of the temperature
            std::uint16_t serial_and_crc;               ///< CRC and serial
        } fields;

        std::uint16_t raw[8]{};
    } prom_{};

    // Interface
    ::SPIDriver* const spi_;
    ::GPIO_TypeDef* const chip_select_port_;
    const std::uint8_t chip_select_pin_;


    template <unsigned IOSize>
    std::array<std::uint8_t, IOSize> io(const std::array<std::uint8_t, IOSize>& tx)
    {
        spiAcquireBus(spi_);
        palClearPad(chip_select_port_, chip_select_pin_);

        std::array<std::uint8_t, IOSize> rx;
        spiExchange(spi_, tx.size(), tx.data(), rx.data());

        palSetPad(chip_select_port_, chip_select_pin_);
        spiReleaseBus(spi_);

        return rx;
    }

    static bool checkPROMCRC(PROM prom)
    {
        const std::uint16_t crc_read = prom.fields.serial_and_crc;

        prom.fields.serial_and_crc = (0xFF00U & (prom.fields.serial_and_crc));

        std::uint16_t n_rem = 0;
        for (std::int16_t cnt = 0; cnt < 16; cnt++)
        {
            if (cnt & 1)
            {
                n_rem ^= std::uint8_t((prom.raw[cnt >> 1]) & 0x00FFU);
            }
            else
            {
                n_rem ^= std::uint8_t(prom.raw[cnt >> 1] >> 8);
            }

            for (std::uint8_t n_bit = 8; n_bit > 0; n_bit--)
            {
                if (n_rem & 0x8000U)
                {
                    n_rem = (n_rem << 1) ^ 0x3000U;
                }
                else
                {
                    n_rem = (n_rem << 1);
                }
            }
        }

        n_rem = (0x000FU & (n_rem >> 12));

        return (0x000FU & crc_read) == (n_rem ^ 0x00U);
    }

    std::uint32_t convertDx(const std::uint8_t cmd)
    {
        // Request conversion
        (void) io(std::array<std::uint8_t, 1>{ cmd });

        ::usleep(10000);     // Wait until ADC conversion is ready

        // Read converted data
        const auto rx = io(std::array<std::uint8_t, 4>{
            Command::ADCRead,
            0,
            0,
            0
        });

        return (std::uint32_t(rx[1]) << 16) | (std::uint32_t(rx[2]) << 8) | rx[3];
    }

public:
    MS5611(::SPIDriver* spi,
           ::GPIO_TypeDef* chip_select_port,
           std::uint8_t chip_select_pin) :
        spi_(spi),
        chip_select_port_(chip_select_port),
        chip_select_pin_(chip_select_pin)
    { }

    /**
     * Performs sensor reset and initialization; can be invoked multiple times.
     */
    bool init()
    {
        // Reset
        (void) io(std::array<std::uint8_t, 1>{ Command::Reset });
        ::usleep(5000);

        // Read PROM
        for (std::uint8_t i = 0; i < 8; i++)
        {
            const auto rx_buf = io(std::array<std::uint8_t, 3>{
                std::uint8_t(Command::PromBase + i * 2),
                0,
                0
            });
            prom_.raw[i] = (rx_buf[1] << 8) | (rx_buf[2]);
        }

        // Check PROM
        if (!checkPROMCRC(prom_))
        {
            os::lowsyslog("MS5611: PROM CRC error\n");
            return false;
        }

        os::lowsyslog("MS5611 PROM:  %5u  %5u  %5u  %5u  %5u  %5u  %5u  %5u\n",
                      prom_.raw[0], prom_.raw[1], prom_.raw[2], prom_.raw[3],
                      prom_.raw[4], prom_.raw[5], prom_.raw[6], prom_.raw[7]);

        return true;
    }

    /**
     * Returns whether the reas was successful and the read sample.
     */
    std::pair<bool, Sample> read()
    {
        static constexpr float ValidPressureRange[] = {
            100,
            200000
        };

        static constexpr float ValidTemperatureRange[] = {
            -50  + DegreesCelsiusToKelvinOffset,
            +100 + DegreesCelsiusToKelvinOffset
        };

        using std::int64_t;

        // Temperature must be converted first because it never changes quickly, so latency is not important;
        // whereas pressure latency is crucial
        const auto temperature_result = convertDx(Command::ConvD2_OSR4096);

        // Temperature offset (in ADC units)
        const int64_t dt = int64_t(temperature_result) - (int64_t(prom_.fields.c5_reference_temp) << 8);

        // Absolute temperature in centidegrees
        int64_t temp = 2000 + ((dt * prom_.fields.c6_temp_coeff_temp) >> 23);

        // Base sensor scale/offset values
        int64_t sens = (int64_t(prom_.fields.c1_pressure_sens) << 15) +
                       ((prom_.fields.c3_temp_coeff_pres_sens * dt) >> 8);

        int64_t offset = (int64_t(prom_.fields.c2_pressure_offset) << 16) +
                         ((prom_.fields.c4_temp_coeff_pres_offset * dt) >> 7);

        // Temperature compensation
        if (temp < 2000)
        {
            const int64_t t2 = (dt * dt) >> 31;
            int64_t f = temp - 2000;
            int64_t off2  = 5 * f >> 1;
            int64_t sens2 = 5 * f >> 2;

            f = f * f;

            if (temp < -1500)
            {
                const int64_t f2 = (temp + 1500) * (temp + 1500);
                off2 += 7 * f2;
                sens2 += 11 * f2 >> 1;
            }

            temp -= t2;
            offset -= off2;
            sens -= sens2;
        }

        // Converting pressure in the last order in order to minimize latency
        const auto pressure_result = convertDx(Command::ConvD1_OSR4096);

        // Computing the pressure
        const int64_t press = (((pressure_result * sens) >> 21) - offset) >> 15;

        // Output
        Sample output;
        output.pressure = float(press);
        output.temperature = float(temp) / 100.0F + DegreesCelsiusToKelvinOffset;

        const bool valid =
            (output.pressure    >= ValidPressureRange[0])    && (output.pressure    <= ValidPressureRange[1]) &&
            (output.temperature >= ValidTemperatureRange[0]) && (output.temperature <= ValidTemperatureRange[1]);

        if (!valid)
        {
            os::lowsyslog("MS5611: Invalid read: %f Pa, %f K\n", output.pressure, output.temperature);
        }

        return {valid, output};
    }
};

}
