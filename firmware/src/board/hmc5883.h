/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 *         Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#pragma once

#include "ch.h"
#include "hal.h"

#if __cplusplus
extern "C" {
#endif

#define HMC_BUF_SIZE      16           /* INPUT BUFFER SIZE */

static const int Hmc5883Addr = 0x1E; /* HMC5883 I2C address */

/**
 * @brief   Configuration register map.
 */
typedef enum
{
    Hmc5883_ConfA = 0x00, /* CONFIGURATION REGISTER A */
    Hmc5883_ConfB = 0x01, /* CONFIGURATION REGISTER B */
    Hmc5883_Mode = 0x02,  /* MODE REGISTER */
    Hmc5883_Xmsb = 0x03,  /* X AXIS VALUE (MSB) REGISTER */
    Hmc5883_Xlsb = 0x04,  /* X AXIS VALUE (LSB) REGISTER */
    Hmc5883_Ymsb = 0x05,  /* Y AXIS VALUE (MSB) REGISTER */
    Hmc5883_Ylsb = 0x06,  /* Y AXIS VALUE (LSB) REGISTER */
    Hmc5883_Zmsb = 0x07,  /* Z AXIS VALUE (MSB) REGISTER */
    Hmc5883_Zlsb = 0x08,  /* Z AXIS VALUE (LSB) REGISTER */
    Hmc5883_Stat = 0x09,  /* STATUS REGISTER */
    Hmc5883_IdA = 0x0A,   /* IDENTIFICATION REGISTER A */
    Hmc5883_IdB = 0x0B,   /* IDENTIFICATION REGISTER B */
    Hmc5883_IdC = 0x0C,   /* IDENTIFICATION REGISTER C */
} hmc5883regs;

/**
 * @brief   Averaging enum.
 */
typedef enum
{
    Avg1samp = 0x00, /* 1 sample per output */
    Avg2samp = 0x01, /* 2 samples per output */
    Avg4samp = 0x02, /* 4 samples per output */
    Avg8samp = 0x03  /* 8 samples per output */
} AveragingVal;

/**
 * @brief   Sample rate enum.
 */
typedef enum
{
    SPS_0p75 = 0x00, /* 0.75 samples per second */
    SPS_1p5 = 0x01,  /* 1.5  samples per second */
    SPS_3 = 0x02,    /* 3    samples per second */
    SPS_7p5 = 0x03,  /* 7.5  samples per second */
    SPS_15 = 0x04,   /* 15   samples per second */
    SPS_30 = 0x05,   /* 30   samples per second */
    SPS_75 = 0x06    /* 75   samples per second */
} SampleRateVal;

/**
 * @brief   Measurement mode enum.
 */
typedef enum
{
    MeasModeNormal = 0x00,  /* Normal mode */
    MeasModePosBias = 0x01, /* Positive bias mode */
    MeasModeNegBias = 0x02  /* Negative bias mode */
} MeasureMode;

/**
 * @brief   Gain enum.
 */
typedef enum
{
    Gain1370 = 0x00, /* 1370 LSBs/Gauss */
    Gain1090 = 0x01, /* 1090 LSBs/Gauss */
    Gain820 = 0x02,  /* 820 LSBs/Gauss */
    Gain660 = 0x03,  /* 660 LSBs/Gauss */
    Gain440 = 0x04,  /* 440 LSBs/Gauss */
    Gain390 = 0x05,  /* 390 LSBs/Gauss */
    Gain330 = 0x06,  /* 330 LSBs/Gauss */
    Gain230 = 0x07   /* 230 LSBs/Gauss */
} GainVal;

/**
 * @brief   Operating mode enum.
 */
typedef enum
{
    OmCont = 0x00, /* Continious measurement mode */
    OmSing = 0x01, /* Single measurement mode */
    OmIdle = 0x02  /* Idle mode */
} OperatingMode;

/**
 * @brief   Main magnetometer parameter structure.
 */
typedef struct
{
    float h[3];  /* magnetic field strenght */
} HMC5883meas_t;

/**
 * @brief   Config register (as struct).
 */
typedef struct
{
    uint8_t regA_ma    :3;  /* number of sample averaged */
    uint8_t regA_do    :3;  /* data output rate register */
    uint8_t regA_ms    :2;  /* measurement configuration */
    uint8_t regB_gain  :3;  /* gain register */
    uint8_t regB_resv  :5;  /* reserve */
    uint8_t regM_speed :6;  /* high speed mode register */
    uint8_t regM_mode  :2;  /* measurement mode */
} HMC5883setup_t;

/**
 * @brief   Config register (as struct and union).
 */
typedef union
{
    uint8_t byte[3];    /* byte representation of HMC5883setup_t structure */
    HMC5883setup_t bit; /* bitfield representation of HMC5883setup_t structure */
} HMC5883set_t;

/**
 * @brief   Main stucture for handling HMC5883 IC
 */
typedef struct
{
    uint8_t buf[HMC_BUF_SIZE]; /* TMP buffer */
    uint32_t hmc5883stat;      /* HMC5883 status */
    HMC5883set_t hmc5883set;   /* HMC5883 setup union*/
} HMC5883_t;

/* Generic functions for HMC5883 */
int16_t hmc5883Init(HMC5883_t *HMC5883, AveragingVal mavg, SampleRateVal drate, MeasureMode mmode, GainVal gain,
                    OperatingMode opmode);
int16_t hmc5883ReadData(HMC5883_t *HMC5883, HMC5883meas_t *Hout);

#if __cplusplus
}
#endif
