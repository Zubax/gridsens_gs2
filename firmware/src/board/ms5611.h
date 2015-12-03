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
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 *         Pavel Kirienko <pavel.kirienko@zubax.com>
 *
 * TODO: Full rewrite in C++.
 */

#pragma once

#include <ch.h>
#include <hal.h>

#if __cplusplus
extern "C"
{
#endif

#pragma pack(push, 1)

/*
 * I2C addresses and commands ENUM
 */
typedef enum
{
    Ms5611Cmd_Reset = 0x1E, /* Reset command for MS5611 */
    /*D1 CONVERSION */
    Ms5611Cmd_ConvD1_OSR256 = 0x40,  /* Convert D1 (oversampling rate = 256) */
    Ms5611Cmd_ConvD1_OSR512 = 0x42,  /* Convert D1 (oversampling rate = 512) */
    Ms5611Cmd_ConvD1_OSR1024 = 0x44, /* Convert D1 (oversampling rate = 1024) */
    Ms5611Cmd_ConvD1_OSR2048 = 0x46, /* Convert D1 (oversampling rate = 2048) */
    Ms5611Cmd_ConvD1_OSR4096 = 0x48, /* Convert D1 (oversampling rate = 4096) */
    /*D2 CONVERSION */
    Ms5611Cmd_ConvD2_OSR256 = 0x50,  /* Convert D2 (oversampling rate = 256) */
    Ms5611Cmd_ConvD2_OSR512 = 0x52,  /* Convert D2 (oversampling rate = 512) */
    Ms5611Cmd_ConvD2_OSR1024 = 0x54, /* Convert D2 (oversampling rate = 1024) */
    Ms5611Cmd_ConvD2_OSR2048 = 0x56, /* Convert D2 (oversampling rate = 2048) */
    Ms5611Cmd_ConvD2_OSR4096 = 0x58, /* Convert D2 (oversampling rate = 4096) */

    Ms5611Cmd_ADCread = 0x00,        /* ADC READ */
    Ms5611Cmd_PromBase = 0xA0        /* PROM BASE ADRRESS 0b1010xxx0 */
} Ms5611Cmd;

/*
 * Calibration PROM as reported by the device.
 */
typedef struct
{
    uint16_t resv;                      /* reserved */
    uint16_t c1_pressure_sens;          /* Pressure sensitivity coef. */
    uint16_t c2_pressure_offset;        /* Pressure offset coef. */
    uint16_t c3_temp_coeff_pres_sens;   /* Temperature coefficient of pressure sensitivity */
    uint16_t c4_temp_coeff_pres_offset; /* Temperature coefficient coefficient of pressure offset */
    uint16_t c5_reference_temp;         /* Reference temperature coef. */
    uint16_t c6_temp_coeff_temp;        /* Temperature coefficient of the temperature */
    uint16_t serial_and_crc;            /* CRC and serial */
} Ms5611Prom;

/*
 * Union representation of Ms5611Prom struct as array of uint16_t
 */
typedef union
{
    uint16_t c[8];
    Ms5611Prom s;
} Ms5611PromUnion;

/*
 * Aux struct for MS5611 control
 */
typedef struct
{
    Ms5611PromUnion ms5611prom;   /* PROM union for MS5611 */
} Ms5611;

#pragma pack(pop)

bool ms5611ReadPT(Ms5611* MS5611, int32_t* pressure, int32_t* temperature);
bool ms5611Reset(Ms5611* MS5611);
bool ms5611GetProm(Ms5611* MS5611);

#if __cplusplus
}
#endif
