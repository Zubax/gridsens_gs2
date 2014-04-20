/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 */

/**
 * @file    hmc5883.c
 * @brief   General functions (driver) for HMC5883 Honeywell magnetoresistive magnetometer.
 *
 */

#include "ch.h"
#include "hal.h"
#include "hmc5883.h"
#include "string.h"


/**
 *  Const float array of coeffs to represent magnetometer Code Units as GAUSSes
 */
const float CUtoG[8] = {0.73E-03,   /* @GAIN_1370 */
                        0.92E-03,   /* @GAIN_1090 */
                        1.22E-03,   /* @GAIN_820 */
                        1.52E-03,   /* @GAIN_660 */
                        2.27E-03,   /* @GAIN_440 */
                        2.56E-03,   /* @GAIN_390 */
                        3.03E-03,   /* @GAIN_330 */
                        4.35E-03};  /* @GAIN_230 */

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
 * @param[out]            TODO: status of setup
 */
int16_t HMC5883_init(HMC5883_t *HMC5883,
                     AveragingVal  mavg,
                     SampleRateVal drate,
                     MeasureMode mmode,
                     GainVal gain,
                     OperatingMode opmode)
{

    HMC5883->HMC5883_set.bit.regA_ma   = mavg;
    HMC5883->HMC5883_set.bit.regA_do   = drate;
    HMC5883->HMC5883_set.bit.regA_ms   = mmode;
    HMC5883->HMC5883_set.bit.regB_gain = gain;
    HMC5883->HMC5883_set.bit.regM_speed = 0;      /* Fixed speed 400 kHz */
    HMC5883->HMC5883_set.bit.regM_mode = opmode;

    /* Copy completed structure to temporary buffer */
    HMC5883->buf[0] = HMC5883reg_CONFA;
    memcpy(&HMC5883->buf[1], &HMC5883->HMC5883_set.byte[0], 0x03); /* Copy init structure to buffer */

        /* Send configuration data to HMC5883_W address */
    i2cAcquireBus(&I2CD2);
        i2cMasterTransmitTimeout(&I2CD2, HMC5883_ADDR, &HMC5883->buf[0], 4, &HMC5883->buf[4], 0, MS2ST(3));
    i2cReleaseBus(&I2CD2);

    chThdSleepMilliseconds(50); /* Wait for ADC conversion is ready */
    /* TODO: check transmit status */

    return 1;
}

/**
 * @brief   Simple init status of
 *
 * @param[out]            TODO: status of setup
 */
int16_t HMC5883_simple_init(void)
{

    static uint8_t initBuf[4]; /* 4 bytes for HMC 5883L Init */
    msg_t  status;

    initBuf[0] = 0x00;
    initBuf[1] = 0x58;
    initBuf[2] = 0x20;
    initBuf[3] = 0x00;

    status = ~RDY_OK;

    while (status != RDY_OK)
    {

    /* Send configuration data to HMC5883_W address */
    i2cAcquireBus(&I2CD2);
        status = i2cMasterTransmitTimeout(&I2CD2, HMC5883_ADDR, &initBuf[0], 4, &initBuf[0], 0, TIME_INFINITE);
    i2cReleaseBus(&I2CD2);

    chThdSleepMilliseconds(50); /* Wait for ADC conversion is ready */
        /* TODO: check transmit status */

    }

    return 1;
}

/**
 * @brief   Set parameters of HMC5883 3 axis magnetic sensor
 *
 * @param[in] *HMC5883    pointer to HMC5883 handling structure
 * @param[in] *Hout       output structure (magnetic field strenght
 * @param[out]            TODO: status of measurement
 */
int16_t HMC5883_readData(HMC5883_t *HMC5883, HMC5883meas_t *Hout)
{
    (void)HMC5883;
    int16_t Hx;               /* AXIS X MEASUREMENT */
    int16_t Hy;               /* AXIS Y MEASUREMENT */
    int16_t Hz;               /* AXIS Z MEASUREMENT */
    msg_t   status;           /* I2C status */
    static uint8_t rx_buf[6]; /* RESCIEVER BUFFER */
    static uint8_t tx_buf[1]; /* TRANSMITTER BUFFER */

    status = ~RDY_OK;

    //tx_buf[0] = HMC5883reg_XMSB;
    tx_buf[0] = 0x03;

    /* Send configuration data to HMC5883_W address */
    i2cAcquireBus(&I2CD2);
        status = i2cMasterTransmitTimeout(&I2CD2, HMC5883_ADDR, &tx_buf[0], 1, &rx_buf[0], 6, TIME_INFINITE);
    i2cReleaseBus(&I2CD2);

    if(status == RDY_OK)
    {

        /* Calculate magnetic field in Code Units */
        Hx = (rx_buf[0] << 8) | rx_buf[1];
        Hy = (rx_buf[2] << 8) | rx_buf[3];
        Hz = (rx_buf[4] << 8) | rx_buf[5];

        /* Scale to Gausses */
        Hout->H[0] = (float)Hx * 0.92E-03; /* CUtoG[HMC5883->HMC5883_set.bit.regB_gain] */;
        Hout->H[1] = (float)Hy * 0.92E-03; /* CUtoG[HMC5883->HMC5883_set.bit.regB_gain] */;
        Hout->H[2] = (float)Hz * 0.92E-03; /* CUtoG[HMC5883->HMC5883_set.bit.regB_gain] */;

    }

    return 1;
}


/** @} */
