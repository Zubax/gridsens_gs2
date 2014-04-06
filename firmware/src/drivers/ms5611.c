/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 */

/**
 * @file    ms5611.c
 * @brief   General functions for ms5611 pressure sensor.
 *
 */

#include "ch.h"
#include "hal.h"

#include "ms5611.h"
#include "string.h"
#include "i2c.h"

/**
 * @brief   MS5611 crc4 cribbed from the datasheet
 *
 * @param[in] *n_prom     pointer to calibration coefficient array
 * @param[out]            calibration status
 */
uint8_t crc4(uint16_t *n_prom){

    int16_t cnt;
    uint16_t n_rem;
    uint16_t crc_read;
    uint8_t n_bit;

    n_rem = 0x00;

    /* save the read crc */
    crc_read = n_prom[7];

    /* remove CRC byte */
    n_prom[7] = (0xFF00 & (n_prom[7]));

    for (cnt = 0; cnt < 16; cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);}

        /* TODO: simplify this terrible code*/
    for (n_bit = 8; n_bit > 0; n_bit--) {
        if (n_rem & 0x8000) {
            n_rem = (n_rem << 1) ^ 0x3000;
        } else {
            n_rem = (n_rem << 1);}}
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
 * @param[in] *MS5611    pointer to MS5611 control structure
 * @param[out] pressure       pointer to pressure data
 * @param[out] tempterature   pointer to temperature data
 */
void MS5611_Read_PT(MS5611_t* MS5611,
                    volatile int32_t* pressure,
                    volatile int32_t* temperature){

//     uint32_t D1;  /* ADC value of the pressure conversion */
//     uint32_t D2;  /* ADC value of the temperature conversion */
//     int32_t dT;   /* difference between actual and measured temp */
//     int64_t OFF;  /* offset at actual temperature */
//     int64_t SENS; /* sensitivity at actual temperature */

//     /* AUX COEFFS */
//     int32_t T2;
//     int64_t OFF2;
//     int64_t SENS2;
//     int64_t TEMP;

//
//     /* Get temperature and pressure */
//     D1 = MS5611_Convert_Dx(MS5611, MS5611_CMD_CONVD1_OSR4096);
//     D2 = MS5611_Convert_Dx(MS5611, MS5611_CMD_CONVD2_OSR4096);
//
//     dT = D2 - (MS5611->MS5611_prom.c[5] << 8);
//     OFF =  (MS5611->MS5611_prom.c[2] << 16) + ((dT * MS5611->MS5611_prom.c[4]) >> 7);
//     SENS = (MS5611->MS5611_prom.c[1] << 15) + ((dT * MS5611->MS5611_prom.c[3]) >> 8);
//     *temperature = 2000 + ((dT * MS5611->MS5611_prom.c[6]) >> 23);

//     /* SECOND ORDER TEMP COMPENSATION */
//
//         /* IF T < 20.0 deg C */
//         if(*temperature < 2000){
//                 T2 = (dT *dT) >> 31;
//         TEMP = *temperature - 2000;
//         OFF2 = (5 *  TEMP * TEMP) >> 1;
//         SENS2 = OFF2 >> 1;
//
//         /* IF T < -15.0 deg C */
//         if(*temperature < -1500){
//                 TEMP = *temperature + 1500;
//                 OFF2 = OFF2 + 7 * TEMP * TEMP;
//                 SENS2 = (SENS2 + (11 * TEMP * TEMP)) >> 1;
//         }

//         *temperature -= T2;
//         OFF -= OFF2;
//         SENS -= SENS2;
//     }

//     *pressure = (int32_t)((((D1 * SENS) >> 21) - OFF) >> 15);

    static uint32_t  D1;  /* ADC value of the pressure conversion */
    static uint32_t  D2;  /* ADC value of the temperature conversion */
    int32_t     dT;  /* difference between actual and measured temp */
    static int32_t  _TEMP;  /* absolute temperature in centidegrees */
    static int32_t  _PRESS; /* absolute temperature in centidegrees */
    int64_t  _SENS;  /* base sensor scale value */
    int64_t   _OFF;  /* base sensor offset value */

    /* Get temperature and pressure */
    D1 = MS5611_Convert_Dx(MS5611, MS5611_CMD_CONVD1_OSR4096);
    D2 = MS5611_Convert_Dx(MS5611, MS5611_CMD_CONVD2_OSR4096);

    /* temperature offset (in ADC units) */
    dT = (int32_t)D2 - ((int32_t)MS5611->MS5611_prom.s.c5_reference_temp << 8);

    /* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
    _TEMP = 2000 + (int32_t)(((int64_t)dT * MS5611->MS5611_prom.s.c6_temp_coeff_temp) >> 23);

    /* base sensor scale/offset values */
    _SENS = ((int64_t)MS5611->MS5611_prom.s.c1_pressure_sens << 15) + (((int64_t)MS5611->MS5611_prom.s.c3_temp_coeff_pres_sens * dT) >> 8);
    _OFF = ((int64_t)MS5611->MS5611_prom.s.c2_pressure_offset << 16) + (((int64_t)MS5611->MS5611_prom.s.c4_temp_coeff_pres_offset * dT) >> 7);

    /* Temperature compensation */
    if (_TEMP < 2000) {

        int32_t T2 = (dT * dT) >> 31;
        int64_t f = (int64_t)_TEMP - 2000;
        int64_t OFF2 = 5 * f >> 1;
        int64_t SENS2 = 5 * f >> 2;

        f = f * f;

        if (_TEMP < -1500) {
            int64_t f2 = (_TEMP + 1500) * (_TEMP + 1500);
            OFF2 += 7 * f2;
            SENS2 += 11 * f2 >> 1;
        }

        _TEMP -= T2;
        _OFF -= OFF2;
        _SENS -= SENS2;
    }

    _PRESS = (((D1 * _SENS) >> 21) - _OFF) >> 15;

    *pressure = _PRESS;
    *temperature = _TEMP;
}

/**
 * @brief   MS5611 reset function.
 *
 * @param[in] *MS5611    pointer to MS5611 control structure
 */
void MS5611_Reset(MS5611_t* MS5611){

    MS5611->status = RDY_OK;
    MS5611->bufTx[0] = MS5611_CMD_RESET; /* Set RESET CMD to transmit buffer */

    /* Sending CMD throught I2C2 */
    i2cAcquireBus(&I2CD2);
    //MS5611->status = i2cMasterTransmitTimeout(&I2CD2, MS5611_ADDR, &MS5611->bufTx[0], 1, &MS5611->bufRx[0], 0, MS2ST(3));
    MS5611->status = i2cMasterTransmitTimeout(&I2CD2, MS5611_ADDR, &MS5611->bufTx[0], 1, &MS5611->bufRx[0], 0, TIME_INFINITE);
    i2cReleaseBus(&I2CD2);


    /* Read and handle errors */
    if(MS5611->status != RDY_OK){
        /* TODO: handle errors */
        MS5611->errors = i2cGetErrors(&I2CD2);
    }
}

/**
 * @brief   MS5611 read coefficient from PROM.
 * @param[in] *MS5611    pointer to MS5611 control structure
 * @param[in] coef_num    coefficient number [0..7]
 * @param[out]            uint16_t coefficient value
 */
uint16_t MS5611_Read_Coef(MS5611_t* MS5611, uint8_t coef_num){

    uint16_t C_tmp = 0; /* Temporary variable for coefficient */

    MS5611->bufTx[0] = MS5611_CMD_PROM_BASE + (coef_num * 2);

    /* TRANSMIT I2C CMD to get PROM coefficient */
    i2cAcquireBus(&I2CD2);
        MS5611->status = i2cMasterTransmitTimeout(&I2CD2, MS5611_ADDR, &MS5611->bufTx[0], 1, MS5611->bufRx, 0, TIME_INFINITE);
    i2cReleaseBus(&I2CD2);

    /* RECEIVE I2C DATA (uint16_t PROM coefficient) */
    i2cAcquireBus(&I2CD2);
        MS5611->status = i2cMasterReceiveTimeout(&I2CD2, MS5611_ADDR, MS5611->bufRx, 2, TIME_INFINITE);
    i2cReleaseBus(&I2CD2);

    /* TODO: get value via pointer */
    C_tmp = (MS5611->bufRx[0] << 8) | MS5611->bufRx[1]; /* Complete uint16_t coef from 2x bytes */

    return C_tmp;
}

/**
 * @brief   MS5611 read PROM and check coeffs with CRC4
 * @param[in] *MS5611    pointer to MS5611 control structure
 * @param[out]           CRC4 correction status (out == 0 => ERROR, out == 1 => OK)
 */
uint8_t MS5611_Get_Prom(MS5611_t* MS5611){

    uint8_t i;        /* Counter value */
    uint8_t promStat; /* PROM status */
    volatile uint16_t tC;  /* Tmp Coef */

    uint8_t txArr[2]; /* Tst TX array */
    uint8_t rxArr[2]; /* Tst RX array */

    /* Read 8x coefficients from prom */
    for(i = 0; i < 8; i++)
    {
        /* Read coefficients from PROM */
        txArr[0] = 0xA0 + (i * 2);

        /* TRANSMIT I2C CMD to get PROM coefficient */
        i2cAcquireBus(&I2CD2);
            //readStat = i2cMasterTransmitTimeout(&I2CD2, MS5611_ADDR, &MS5611->bufTx[0], 1, MS5611->bufRx, 0, TIME_INFINITE);
                  i2cMasterTransmitTimeout(&I2CD2, MS5611_ADDR, txArr, 1, rxArr, 0, TIME_INFINITE);
        i2cReleaseBus(&I2CD2);

              /* RECEIVE I2C DATA (uint16_t PROM coefficient) */
                i2cAcquireBus(&I2CD2);
            //i2cMasterReceiveTimeout(&I2CD2, MS5611_ADDR, MS5611->bufRx, 2, TIME_INFINITE);
                  i2cMasterReceiveTimeout(&I2CD2, MS5611_ADDR, rxArr, 2, TIME_INFINITE);
        i2cReleaseBus(&I2CD2);

              tC = (rxArr[0] << 8) | (rxArr[1]);
        MS5611->MS5611_prom.c[i] = tC; /* Read coef */
    }

        promStat = crc4(&MS5611->MS5611_prom.c[0]); /* Calculate CRC and read status */

        /* Fill LSB in MS5611_stat register with promStat */
        MS5611->MS5611_stat &= ~1; /* Clear LSB */
        MS5611->MS5611_stat |= promStat;

        return promStat; /* promStat == 1 => CRC4 is GOOD */
}

/**
 * @brief   MS5611 read converted value of TEMPERATURE or PRESSURE.
 * @param[in] *MS5611    pointer to MS5611 control structure
 * @param[in] coef_num    coefficient number [0..7]
 * @param[out]            uint16_t coefficient value
 */
uint32_t MS5611_Convert_Dx(MS5611_t* MS5611, ms5611_cmd cmd){

    static uint32_t Dx = 0;
    uint8_t i = 0;

    uint8_t TXarr[1];
    uint8_t RXarr[3];

    /* TODO: fill with 0x00 with memset or not clear rxbuffer if not need it */
    for(i = 0; i < 3; i++)
        RXarr[i] = 0;

        /* Sending CMD throught I2C2 */
    TXarr[0] = (uint8_t)cmd;

    i2cAcquireBus(&I2CD2);
        MS5611->status = i2cMasterTransmitTimeout(&I2CD2, MS5611_ADDR, &TXarr[0], 1, &RXarr[0], 0, MS2ST(3));
    i2cReleaseBus(&I2CD2);

    chThdSleepMilliseconds(50); /* Wait for ADC conversion is ready */

    /* Read converted data */
    TXarr[0] = (uint8_t)MS5611_CMD_ADCREAD;

    i2cAcquireBus(&I2CD2);
        MS5611->status = i2cMasterTransmitTimeout(&I2CD2, MS5611_ADDR, &TXarr[0], 1, &RXarr[0], 0, MS2ST(3));
    i2cReleaseBus(&I2CD2);

    /* RECEIVE I2C DATA (uint16_t PROM coefficient) */
    i2cAcquireBus(&I2CD2);
        MS5611->status = i2cMasterReceiveTimeout(&I2CD2, MS5611_ADDR, &RXarr[0], 3, MS2ST(3));
    i2cReleaseBus(&I2CD2);

    /* Collect uint32_t from 3x bytes */
    Dx = (RXarr[0] << 16) | (RXarr[1] << 8) | (RXarr[2]);

    return Dx;
}
