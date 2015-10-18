/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 *         Pavel Kirienko <pavel.kirienko@zubax.com>
 *
 * TODO: Full rewrite in C++.
 */

/**
 * @file    ms5611.c
 * @brief   General functions for ms5611 pressure sensor.
 *
 */

#include "ms5611.h"
#include <i2c.h>
#include <string.h>

#define Ms5611BufSize 16         /* define size of input and output buffers in MS5611 aux structure */
#define MS5611Addr    0x77       /* MS5611 adress @ CS pin connected with GND */

static bool convertDx(Ms5611Cmd cmd, uint32_t* out);

/**
 * @brief   MS5611 crc4 cribbed from the datasheet
 *
 * @param[in] *n_prom     pointer to calibration coefficient array
 * @return                calibration status
 */
static bool crc4(uint16_t *n_prom)
{
    uint16_t n_rem = 0;

    /* save the read crc */
    const uint16_t crc_read = n_prom[7];

    /* remove CRC byte */
    n_prom[7] = (0xFF00 & (n_prom[7]));

    for (int16_t cnt = 0; cnt < 16; cnt++)
    {
        /* uneven bytes */
        if (cnt & 1)
        {
            n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);
        }
        else
        {
            n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
        }

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & 0x8000)
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }

    /* final 4 bit remainder is CRC value */
    n_rem = (0x000F & (n_rem >> 12));
    n_prom[7] = crc_read;

    /* return true if CRCs match */
    return (0x000F & crc_read) == (n_rem ^ 0x00);
}

/**
 * @brief   MS5611 read and calculate temperature and pressure
 *
 * @param[in] *ms5611         pointer to MS5611 control structure
 * @param[out] pressure       pointer to pressure data
 * @param[out] tempterature   pointer to temperature data
 * @return                    true on success
 */
bool ms5611ReadPT(Ms5611* ms5611, int32_t* pressure, int32_t* temperature)
{
    /* Get temperature and pressure */
    uint32_t d1 = 0; /* ADC value of the pressure conversion */
    uint32_t d2 = 0; /* ADC value of the temp conversion */

    if (!convertDx(Ms5611Cmd_ConvD1_OSR4096, &d1))
    {
        return false;
    }
    if (!convertDx(Ms5611Cmd_ConvD2_OSR4096, &d2))
    {
        return false;
    }

    /* temperature offset (in ADC units) */
    const int32_t dt = (int32_t)d2 - ((int32_t)ms5611->ms5611prom.s.c5_reference_temp << 8);

    /* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
    int32_t _temp = 2000 + (int32_t)(((int64_t)dt * ms5611->ms5611prom.s.c6_temp_coeff_temp) >> 23);

    /* base sensor scale/offset values */
    int64_t _sens = ((int64_t)ms5611->ms5611prom.s.c1_pressure_sens << 15) +
                    (((int64_t)ms5611->ms5611prom.s.c3_temp_coeff_pres_sens * dt) >> 8);

    int64_t _off = ((int64_t)ms5611->ms5611prom.s.c2_pressure_offset << 16) +
                   (((int64_t)ms5611->ms5611prom.s.c4_temp_coeff_pres_offset * dt) >> 7);

    /* Temperature compensation */
    if (_temp < 2000)
    {
        const int32_t t2 = (dt * dt) >> 31;
        int64_t f = (int64_t)_temp - 2000;
        int64_t off2  = 5 * f >> 1;
        int64_t sens2 = 5 * f >> 2;

        f = f * f;

        if (_temp < -1500)
        {
            int64_t f2 = (_temp + 1500) * (_temp + 1500);
            off2 += 7 * f2;
            sens2 += 11 * f2 >> 1;
        }

        _temp -= t2;
        _off -= off2;
        _sens -= sens2;
    }

    const int32_t _press = (((d1 * _sens) >> 21) - _off) >> 15;

    *pressure = _press;
    *temperature = _temp;
    return true;
}

/**
 * @brief   MS5611 reset function.
 *
 * @param[in] *MS5611    pointer to MS5611 control structure
 * @return               true on success
 */
bool ms5611Reset(Ms5611* ms5611)
{
    (void)ms5611;

    uint8_t buf_tx[] = { Ms5611Cmd_Reset };
    msg_t status = RDY_OK;

    i2cAcquireBus(&I2CD1);
    {
        status = i2cMasterTransmitTimeout(&I2CD1, MS5611Addr, &buf_tx[0], 1, &buf_tx[0], 0, MS2ST(5));
    }
    i2cReleaseBus(&I2CD1);

    chThdSleepMilliseconds(5);
    return status == RDY_OK;
}

/**
 * @brief   MS5611 read PROM and check coeffs with CRC4
 * @param[in] *MS5611    pointer to MS5611 control structure
 * @return               CRC4 status (out == 0 => ERROR, out == 1 => OK)
 */
bool ms5611GetProm(Ms5611* ms5611)
{
    /* Read 8x coefficients from prom */
    for (uint8_t i = 0; i < 8; i++)
    {
        const uint8_t tx_arr[] = { Ms5611Cmd_PromBase + (i * 2) };
        uint8_t rx_arr[] = { 0x00, 0x00 };
        msg_t status = ~RDY_OK;

        i2cAcquireBus(&I2CD1);
        {
            status = i2cMasterTransmitTimeout(&I2CD1, MS5611Addr, tx_arr, 1, rx_arr, 2, MS2ST(5));
        }
        i2cReleaseBus(&I2CD1);

        if (status != RDY_OK)
        {
            return false;
        }

        const uint16_t tc = (rx_arr[0] << 8) | (rx_arr[1]); /* Temp coef */
        ms5611->ms5611prom.c[i] = tc; /* Read coef */
    }

    return crc4(&ms5611->ms5611prom.c[0]); /* Calculate CRC and read status; true if OK */
}

/**
 * @brief   MS5611 read converted value of TEMPERATURE or PRESSURE.
 * @param[in] coef_num    coefficient number [0..7]
 * @param[out]            uint16_t coefficient value
 * @return                true on success
 */
static bool convertDx(Ms5611Cmd cmd, uint32_t* out)
{
    if (out == NULL)
    {
        return false;
    }

    uint8_t rx_arr[] = { 0, 0, 0 };
    msg_t status = ~RDY_OK;

    /*
     * Request conversion
     */
    i2cAcquireBus(&I2CD1);
    {
        const uint8_t tx_arr[] = { cmd };
        status = i2cMasterTransmitTimeout(&I2CD1, MS5611Addr, tx_arr, 1, rx_arr, 0, MS2ST(5));
    }
    i2cReleaseBus(&I2CD1);

    if (status != RDY_OK)
    {
        return false;
    }

    chThdSleepMilliseconds(10); /* Wait until ADC conversion is ready */

    /*
     * Read converted data
     */
    i2cAcquireBus(&I2CD1);
    {
        const uint8_t tx_arr[] = { Ms5611Cmd_ADCread };
        status = i2cMasterTransmitTimeout(&I2CD1, MS5611Addr, tx_arr, 1, rx_arr, 3, MS2ST(5));
    }
    i2cReleaseBus(&I2CD1);

    if (status != RDY_OK)
    {
        return false;
    }

    *out = (((uint32_t)rx_arr[0]) << 16) | (((uint32_t)rx_arr[1]) << 8) | (rx_arr[2]);

    /* Datasheet says that in case of incomplete conversion Dx will be zero */
    return *out != 0;
}
