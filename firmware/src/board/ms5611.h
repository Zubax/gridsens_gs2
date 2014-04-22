/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 *         Pavel Kirienko <pavel.kirienko@courierdrone.com>
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
typedef struct prom_s
{
    uint16_t resv;                      /* reserved */
    uint16_t c1_pressure_sens;          /* Pressure sensitivity coef. */
    uint16_t c2_pressure_offset;        /* Pressure offset coef. */
    uint16_t c3_temp_coeff_pres_sens;   /* Temperature coefficient of pressure sensitivity */
    uint16_t c4_temp_coeff_pres_offset; /* Temperature coefficient coefficient of pressure offset */
    uint16_t c5_reference_temp;         /* Reference temperature coef. */
    uint16_t c6_temp_coeff_temp;        /* Temperature coefficient of the temperature */
    uint16_t serial_and_crc;            /* CRC and serial */
} prom_s;

/*
 * Union representation of prom_s struct as array of uint16_t
 */
typedef union prom_u
{
    uint16_t c[8];
    prom_s s;
} prom_u;

/*
 * Aux struct for MS5611 control
 */
typedef struct MS5611_t
{
    prom_u ms5611prom;   /* PROM union for MS5611 */
} MS5611_t;

/*
 * Temperature and pressure measurement structure
 */
typedef struct TP_t
{
    float p;  /* Atmospheric pressure [Pa] */
    float t;  /* Temperature [Deg C] */
} TP_t;

#pragma pack(pop)

bool ms5611ReadPT(MS5611_t* MS5611, int32_t* pressure, int32_t* temperature);
bool ms5611Reset(MS5611_t* MS5611);
bool ms5611GetProm(MS5611_t* MS5611);

#if __cplusplus
}
#endif
