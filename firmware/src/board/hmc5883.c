/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 *         Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

/**
 * @file    hmc5883.c
 * @brief   General functions (driver) for HMC5883 Honeywell magnetoresistive magnetometer.
 *
 */

#include "hmc5883.h"
#include <ch.h>
#include <hal.h>
#include <string.h>

/**
 * Const float array of coeffs to represent magnetometer Code Units as GAUSSes
 */
const float CUtoG[8] =
{
    0.73E-03, /* @GAIN_1370 */
    0.92E-03, /* @GAIN_1090 */
    1.22E-03, /* @GAIN_820 */
    1.52E-03, /* @GAIN_660 */
    2.27E-03, /* @GAIN_440 */
    2.56E-03, /* @GAIN_390 */
    3.03E-03, /* @GAIN_330 */
    4.35E-03  /* @GAIN_230 */
};

/**
 * @brief   Set parameters of HMC5883 3 axis magnetic sensor
 *
 * @param[in] *HMC5883    pointer to HMC5883 handling structure
 * @param[in] mavg        averaging setup
 * @param[in] drate       data rate setup
 * @param[in] mmode       measurement mode setup
 * @param[in] gain        gain setup
 * @param[in] hspeed      high speed setup
 * @param[in] opmode      operating mode setup
 * @return
 */
int16_t hmc5883Init(HMC5883_t *HMC5883, AveragingVal mavg, SampleRateVal drate, MeasureMode mmode, GainVal gain,
                    OperatingMode opmode)
{
    (void)mavg;
    (void)drate;
    (void)mmode;
    (void)gain;
    (void)opmode;
//    HMC5883->hmc5883set.bit.regA_ma = mavg;
//    HMC5883->hmc5883set.bit.regA_do = drate;
//    HMC5883->hmc5883set.bit.regA_ms = mmode;
//    HMC5883->hmc5883set.bit.regB_gain = gain;
//    HMC5883->hmc5883set.bit.regM_speed = 0; /* Fixed speed 400 kHz */
//    HMC5883->hmc5883set.bit.regM_mode = opmode;

    HMC5883->hmc5883set.byte[0] = 0x78; // 0b01111000
    HMC5883->hmc5883set.byte[1] = 0x20; // 0b00100000
    HMC5883->hmc5883set.byte[2] = 0x00;

    HMC5883->buf[0] = Hmc5883_ConfA;
    memcpy(&HMC5883->buf[1], &HMC5883->hmc5883set.byte[0], 0x03); /* Copy init structure to buffer */

    msg_t status = ~RDY_OK;
    i2cAcquireBus(&I2CD2);
    {
        status = i2cMasterTransmitTimeout(&I2CD2, Hmc5883Addr, HMC5883->buf, 4, HMC5883->buf, 0, MS2ST(10));
    }
    i2cReleaseBus(&I2CD2);

    chThdSleepMilliseconds(10); /* Wait until ADC conversion is ready */
    return (status == RDY_OK) ? 1 : -1;
}

/**
 * @brief   Set parameters of HMC5883 3 axis magnetic sensor
 *
 * @param[in] *HMC5883    pointer to HMC5883 handling structure
 * @param[in] *Hout       output structure (magnetic field strenght
 * @return                TODO: status of measurement
 */
int16_t hmc5883ReadData(HMC5883_t *HMC5883, HMC5883meas_t *Hout)
{
    uint8_t rx_buf[6] = {0, 0, 0, 0, 0, 0};
    uint8_t tx_buf[1];

    tx_buf[0] = Hmc5883_Xmsb;

    msg_t status = ~RDY_OK;

    i2cAcquireBus(&I2CD2);
    {
        status = i2cMasterTransmitTimeout(&I2CD2, Hmc5883Addr, &tx_buf[0], 1, &rx_buf[0], 6, MS2ST(10));
    }
    i2cReleaseBus(&I2CD2);

    if (status == RDY_OK)
    {
        /* Calculate magnetic field in Code Units */
        const int16_t hx = (((int16_t)rx_buf[0]) << 8) | rx_buf[1];  // X
        const int16_t hz = (((int16_t)rx_buf[2]) << 8) | rx_buf[3];  // Z (!!)
        const int16_t hy = (((int16_t)rx_buf[4]) << 8) | rx_buf[5];  // Y (!!)

        /* Scale to Gausses TODO: apply proper scale */
        (void)HMC5883;
        Hout->h[0] = ((float)hx);// * CUtoG[HMC5883->hmc5883set.bit.regB_gain];
        Hout->h[1] = ((float)hy);// * CUtoG[HMC5883->hmc5883set.bit.regB_gain];
        Hout->h[2] = ((float)hz);// * CUtoG[HMC5883->hmc5883set.bit.regB_gain];
    }
    return 1;
}
