/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 */

#ifndef _ms5611_h
#define _ms5611_h

#include "ch.h"
#include "hal.h"

/**
 * @name    ms5611.h header files
 * @{
 */

#define MS5611_BUFSIZE 16         /* define size of input and output buffers in MS5611 aux structure */
#define MS5611_ADDR    0x77       /* MS5611 adress @ CS pin connected with GND */

#pragma pack(push, 1)

/*
* I2C addresses and commands ENUM
*/
typedef enum {
    MS5611_CMD_RESET          = 0x1E,   /* Reset command for MS5611 */
    /*D1 CONVERSION */
    MS5611_CMD_CONVD1_OSR256  = 0x40,   /* Convert D1 (oversampling rate = 256) */
    MS5611_CMD_CONVD1_OSR512  = 0x42,   /* Convert D1 (oversampling rate = 512) */
    MS5611_CMD_CONVD1_OSR1024 = 0x44,   /* Convert D1 (oversampling rate = 1024) */
    MS5611_CMD_CONVD1_OSR2048 = 0x46,   /* Convert D1 (oversampling rate = 2048) */
    MS5611_CMD_CONVD1_OSR4096 = 0x48,   /* Convert D1 (oversampling rate = 4096) */
    /*D2 CONVERSION */
    MS5611_CMD_CONVD2_OSR256  = 0x50,   /* Convert D2 (oversampling rate = 256) */
    MS5611_CMD_CONVD2_OSR512  = 0x52,   /* Convert D2 (oversampling rate = 512) */
    MS5611_CMD_CONVD2_OSR1024 = 0x54,   /* Convert D2 (oversampling rate = 1024) */
    MS5611_CMD_CONVD2_OSR2048 = 0x56,   /* Convert D2 (oversampling rate = 2048) */
    MS5611_CMD_CONVD2_OSR4096 = 0x58,   /* Convert D2 (oversampling rate = 4096) */

    MS5611_CMD_ADCREAD        = 0x00,   /* ADC READ */
    MS5611_CMD_PROM_BASE      = 0xA0,   /* PROM BASE ADRRESS 1010xxx0 */
} ms5611_cmd;

/*
* Calibration PROM as reported by the device.
*/
typedef struct prom_s {
    uint16_t resv;                       /* reserved */
    uint16_t c1_pressure_sens;           /* Pressure sensitivity coef. */
    uint16_t c2_pressure_offset;         /* Pressure offset coef. */
    uint16_t c3_temp_coeff_pres_sens;    /* Temperature coefficient of pressure sensitivity */
    uint16_t c4_temp_coeff_pres_offset;  /* Temperature coefficient coefficient of pressure offset */
    uint16_t c5_reference_temp;          /* Reference temperature coef. */
    uint16_t c6_temp_coeff_temp;         /* Temperature coefficient of the temperature */
    uint16_t serial_and_crc;             /* CRC and serial */
}prom_s;

/*
* Union representation of prom_s struct as array of uint16_t
*/
typedef union prom_u {
    uint16_t c[8];
    prom_s s;
}prom_u;

/*
*  Aux struct for MS5611 control
*/
typedef struct MS5611_t
{
    uint8_t  bufRx[MS5611_BUFSIZE];    /* TMP Rx buffer */
    uint8_t  bufTx[MS5611_BUFSIZE];    /* TMP Tx buffer */
    msg_t    status;                   /* I2C Message status struct */
    i2cflags_t errors;                 /* I2C Error flag structure */

    uint32_t MS5611_stat;              /* MS5611 status/control register */
    prom_u   MS5611_prom;              /* PROM union for MS5611 */
}MS5611_t;

/*
*  Temperature and pressure measurement structure
*/
typedef struct TP_t
{
    float P;                           /* Atmospheric pressure [Pa] */
    float Pdisp;                       /* Dispersion of atmospheric pressure */
    float T;                           /* Temperature [Deg C] */
    float Tdisp;                       /* Dispersion of temperature */
}TP_t;

#pragma pack(pop)


/* Function desctiption */
uint8_t crc4(uint16_t *n_prom);
void MS5611_Read_PT(MS5611_t* MS5611, volatile int32_t* pressure, volatile int32_t* temperature);
void MS5611_Reset(MS5611_t* MS5611);
uint16_t MS5611_Read_Coef(MS5611_t* MS5611, uint8_t coef_num);
uint8_t MS5611_Get_Prom(MS5611_t* MS5611);
uint32_t MS5611_Convert_Dx(MS5611_t* MS5611, ms5611_cmd cmd);

#endif /* _ms5611_h */

/** @} */
