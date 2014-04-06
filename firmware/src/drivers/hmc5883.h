/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 */

#ifndef _hmc5883_h
#define _hmc5883_h

#include "ch.h"
#include "hal.h"

/**
 * @name    hmc5883.h header files
 * @{
 */
#define HMC_BUF_SIZE     16   /* INPUT BUFFER SIZE */
#define HMC5883_ADDR     0x1E /* HMC5883 I2C address */

/**
 * @brief   Configuration register map.
 * @note    none
 */
typedef enum {
    HMC5883reg_CONFA = 0x00,     /* CONFIGURATION REGISTER A */
    HMC5883reg_CONFB = 0x01,     /* CONFIGURATION REGISTER B */
    HMC5883reg_MODE =  0x02,     /* MODE REGISTER */
    HMC5883reg_XMSB =  0x03,     /* X AXIS VALUE (MSB) REGISTER */
    HMC5883reg_XLSB =  0x04,     /* X AXIS VALUE (LSB) REGISTER */
    HMC5883reg_YMSB =  0x05,     /* Y AXIS VALUE (MSB) REGISTER */
    HMC5883reg_YLSB =  0x06,     /* Y AXIS VALUE (LSB) REGISTER */
    HMC5883reg_ZMSB =  0x07,     /* Z AXIS VALUE (MSB) REGISTER */
    HMC5883reg_ZLSB =  0x08,     /* Z AXIS VALUE (LSB) REGISTER */
    HMC5883reg_STAT =  0x09,     /* STATUS REGISTER */
    HMC5883reg_ID_A =  0x0A,     /* IDENTIFICATION REGISTER A */
    HMC5883reg_ID_B =  0x0B,     /* IDENTIFICATION REGISTER B */
    HMC5883reg_ID_C =  0x0C,     /* IDENTIFICATION REGISTER A */
} hmc5883regs;

/**
 * @brief   Averaging enum.
 * @note    none
 */
typedef enum {
    AVG_1SPS = 0x00,     /* 1 sample per output */
    AVG_2SPS = 0x01,     /* 2 samples per output */
    AVG_4SPS = 0x02,     /* 4 samples per output */
    AVG_8SPS = 0x03,     /* 8 samples per output */
} AveragingVal;

/**
 * @brief   Sample rate enum.
 * @note    none
 */
typedef enum {
    SPS_0p75 = 0x00,   /* 0.75 samples per second */
    SPS_1p5 = 0x01,    /* 1.5  samples per second */
    SPS_3 = 0x02,      /* 3    samples per second */
    SPS_7p5 = 0x03,    /* 7.5  samples per second */
    SPS_15 = 0x04,     /* 15   samples per second */
    SPS_30 = 0x05,     /* 30   samples per second */
    SPS_75 = 0x06,     /* 75   samples per second */
} SampleRateVal;

/**
 * @brief   Measurement mode enum.
 * @note    none
 */
typedef enum {
    MM_normal  = 0x00,   /* Normal mode */
    MM_posbias = 0x01,   /* Positive bias mode */
    MM_negbias = 0x02,   /* Negative bias mode */
} MeasureMode;

/**
 * @brief   Gain enum.
 * @note    none
 */
typedef enum {
    GAIN_1370 = 0x00,   /* 1370 LSBs/Gauss */
    GAIN_1090 = 0x01,   /* 1090 LSBs/Gauss */
    GAIN_820  = 0x02,   /* 820 LSBs/Gauss */
    GAIN_660  = 0x03,   /* 660 LSBs/Gauss */
    GAIN_440  = 0x04,   /* 440 LSBs/Gauss */
    GAIN_390  = 0x05,   /* 390 LSBs/Gauss */
    GAIN_330  = 0x06,   /* 330 LSBs/Gauss */
    GAIN_230  = 0x07,   /* 230 LSBs/Gauss */
} GainVal;

/**
 * @brief   Operating mode enum.
 * @note    none
 */
typedef enum {
    OM_sing  = 0x00,  /* Single measurement mode */
    OM_cont = 0x01,   /* Continious measurement mode */
    OM_idle = 0x02,   /* Idle mode */
} OperatingMode;

/**
 * @brief   Main magnetometer parameter structure.
 * @note    none
 */
typedef struct HMC5883meas_t
{
    float H[3];          /* magnetic field strenght */
    float Hc[9];         /* measurement covariation matrix */
}HMC5883meas_t;

/**
 * @brief   Config register (as struct).
 * @note    none
 */
typedef struct HMC5883setup_t
{
    uint8_t regA_ma : 3,     /* number of sample averaged */
            regA_do : 3,     /* data output rate register */
            regA_ms : 2;     /* measurement configuration */
    uint8_t regB_gain : 3,   /* gain register */
            regB_resv : 5;   /* reserve */
    uint8_t regM_speed : 6,  /* high speed mode register */
            regM_mode : 2;   /* measurement mode */
}HMC5883setup_t;

/**
 * @brief   Config register (as struct and union).
 * @note    none
 */
typedef union HMC5883set_t {
    uint8_t        byte[3]; /* byte representation of HMC5883setup_t structure */
    HMC5883setup_t bit;     /* bitfield representation of HMC5883setup_t structure */ 
}HMC5883set_t;

/**
 * @brief   Main stucture for handling HMC5883 IC
 * @note    none
 */
typedef struct HMC5883_t
{
	  uint8_t        buf[HMC_BUF_SIZE];  /* TMP buffer */
	
	  uint32_t       HMC5883_stat;       /* HMC5883 status */
	  HMC5883set_t   HMC5883_set;        /* HMC5883 setup union*/
}HMC5883_t;

/* Generic functions for HMC5883 */
int16_t HMC5883_init(HMC5883_t *HMC5883, AveragingVal mavg, SampleRateVal drate, MeasureMode mmode, GainVal gain, OperatingMode opmode);
int16_t HMC5883_simple_init(void);
int16_t HMC5883_readData(HMC5883_t *HMC5883, HMC5883meas_t *Hout);

#endif /* _hmc5883_h */

/** @} */
