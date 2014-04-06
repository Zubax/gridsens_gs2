/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 */

/**
 * @file    ublox.c
 * @brief   General functions for UBLOX (i.e. MAX 7) GNSS reciever.
 *
 */

#include "ch.h"
#include "hal.h"
#include "ublox.h"
#include "string.h"

/**
 * @brief   Form UBX message to uint8_t array
 *
 * @param[in] msgbuf[]    pointer to buffer (during saving data)
 * @param[in] msg_class   UBX message class
 * @param[in] msg_id      UBX message ID
 * @param[in] msg         pointer to UBX message body
 * @param[in] data_len    lenght of message body
 * @param[out]            total number of bytes in output array
 */
uint16_t ubx_write(uint8_t  *msgbuf,
                                 uint8_t  msg_class,
                                 uint8_t  msg_id,
                     uint8_t  *msg,
                                 uint16_t data_len)
{
    uint8_t CK_A, CK_B;
    uint16_t i;

    /*@ -type @*/
    msgbuf[0] = 0xb5;
    msgbuf[1] = 0x62;

    CK_A = CK_B = 0;
    msgbuf[2] = msg_class;
    msgbuf[3] = msg_id;
    msgbuf[4] = data_len & 0xff;
    msgbuf[5] = (data_len >> 8) & 0xff;

        /* check for presence of data */
    if (msg != NULL){
          memcpy(&msgbuf[6], msg, data_len);
    }

    /* calculate CRC */
    for (i = 2; i < 6; i++) {
        CK_A += msgbuf[i];
        CK_B += CK_A;
    }

    /*@ -nullderef @*/
    if (msg != NULL)
      for (i = 0; i < data_len; i++) {
        CK_A += msg[i];
        CK_B += CK_A;
      }

    msgbuf[6 + data_len] = CK_A;
    msgbuf[7 + data_len] = CK_B;

    /* SEND DATAT TO PORT (CHIBOS SERIAL PORT MUST BE CONFIGURATED) */
    sdWrite(&SD2, msgbuf, data_len + 8);    /* WRITE DATA TO SERIAL PORT */

    /*  COMMENT:
            BLOCKING WRITE METHOD BECAUSE SERIAL2 (UBLOX) TX
            IS USED ONLY DURING INIT */

    return (data_len + 8);
}

/**
 * @brief   Find UBX message header in uint8_t array
 *
 * @param[in] *buf        pointer to buffer (to find UBX header)
 * @param[in] len         length of input buffer
 * @param[in] stpos       start position of search
 * @param[out]            number of position of first UBX message in buffer (if header isn't find => index == -1)
 */
int16_t ubx_findheader(uint8_t *buf, uint16_t len, uint16_t st_pos)
{
    uint16_t i;                    /* counter variable */
    int16_t data_hind;  /* position of first header index */

    data_hind = -1;

    /* Try to find UBX header */
    for(i = st_pos; i < len - st_pos - 1; i++){
        if(buf[i] == 0xB5){
            if(buf[i+1] == 0x62){
                data_hind = i;
            break;}
        }
    }

    /* Return first index number */
    return data_hind;
}

/**
 * @brief   Calculate UBX CRC in uint8_t array
 *
 * @param[in] *buf        pointer to buffer (to calculate UBX CRC)
 * @param[in] len         length of input buffer
 * @param[out]            returns CRC of buffer
 */
uint16_t ubx_calcCRC(uint8_t *buf, uint16_t len)
{
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    uint16_t i;
    uint16_t crc;

    /* Calculate CRC */
    for(i = 0; i < len; i++)
    {
        CK_A += buf[i];
        CK_B += CK_A;
    }

    crc = (CK_A << 8) | CK_B;

    /* Return CRC */
    return crc;
}

/**
 * @brief   Calculate UBX CRC in uint8_t array
 *
 * @param[in] *GNSS_t     pointer to GNSS structure (to update GNSS parameters)
 * @param[in] *buf        pointer to buffer (to parse UBX packet)
 * @param[in] len         length of input buffer
 * @param[out]            returns size of parsed UBX message (negative = UBX parsing error)
 */
int16_t ubx_parse(t_GNSS *GNSS_t,
                  uint8_t *buf,
                  uint16_t len)
{
    static uint16_t data_len;    /* total lenght of data */
    static uint16_t msgid;     /* message ID */

    /* The packet at least contains a head long enough for an empty message */
    if (len < UBX_PREFIX_LEN)
        return -1;    /* Parse ERROR: not sufficient data */

    /* Extract message id and length */
    msgid = (buf[2] << 8) | buf[3];
    data_len = (size_t) getles16(buf, 4);

    if (len < data_len + 8)
    return -1;    /* Parse ERROR: not sufficient data */

    /* TODO: Get checksum of packet */
    /* tCRC = (buf[data_len + 6] << 8) | buf[data_len + 7]; */

    /* Decode packet & update GNSS data */
    switch (msgid) {
        case UBX_NAV_DOP: /* UBX_NAV_DOP */
            ubx_msg_nav_dop(GNSS_t, &buf[6], data_len);     /* NAV DOP EXECUTION */
            break;
        case UBX_NAV_SOL: /* UBX NAV SOL */
            ubx_msg_nav_sol(GNSS_t, &buf[6], data_len);     /* NAV SOL EXECUTION */
            break;
        case UBX_NAV_POSLLH: /* UBX NAV POSLLH */
            ubx_msg_nav_posllh(GNSS_t, &buf[6], data_len);  /* NAV POSLLH EXECUTION */
            break;
        case UBX_NAV_PVT: /* UBX NAV PVT */
            ubx_msg_nav_pvt(GNSS_t, &buf[6], data_len);     /* NAV PVT EXECUTION */
            break;
        case UBX_NAV_SVINFO:
            ubx_msg_nav_svinfo(GNSS_t, &buf[6], data_len);  /* NAV SVINFO EXECUTION */
            break;
        default:
            break;
    }

    return (data_len + 8); /* Parsing of packet is complete */

}

/**
 * @brief    Set required msg types to recieve (UBX MODE)
 */
void ubx_setMSGtypes(t_GNSS *GNSS) /* DUE TO u-Center UBX example */
{
    uint8_t  msg[8];

    /* SETUP BINARY MODE OUTPUT RATES */

    /* PRE HEADER */
    msg[0] = 0x01;        /* class */
    msg[1] = 0x04;        /* msg id  = UBX_NAV_DOP */

    msg[2] = 0x01;        /* rate I2C ? */
    msg[3] = 0x01;        /* rate UART1 */
    msg[4] = 0x01;        /* rate UART2 */
    msg[5] = 0x01;        /* rate USB */
    msg[6] = 0x01;        /* rate SPI */
    msg[7] = 0x00;        /* ??? */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 8);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x06;        /* msg id  = NAV_SOL */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 8);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x02;        /* msg id  = UBX_NAV_POSLLH */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 8);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x07;        /* msg id  = UBX_NAV_PVT */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 8);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x30;        /* msg id  = NAV-SVINFO */

    msg[2] = 0x0a;        /* rate I2C ? */
    msg[3] = 0x0a;        /* rate UART1 */
    msg[4] = 0x0a;        /* rate UART2 */
    msg[5] = 0x0a;        /* rate USB */
    msg[6] = 0x0a;        /* rate SPI */

    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 8);
}


/**
 * @brief   Init UBX mode with known baudrate, parity bits setup, stopbits and mode
 *
 * @param[in] baudrate    uart port baudrate setup
 * @param[in] parity      parity bits setup
 * @param[in] stopbits    stopbits setup
 * @param[in] mode        mode setup
 */
void ubx_init(t_GNSS *GNSS,
                uint32_t baudrate,
                uint8_t parity,
                uint8_t stopbits,
                uint32_t mode)
{
    uint32_t usart_mode = 0;
    uint8_t  buf[UBX_CFG_LEN];
    uint8_t  msg[3];

    CFG_NAV5 nav5_cfg; /* NAV5_cfg init structure */
    CFG_RATE rate_cfg; /* RATE_cfg init structure */

    /* SETUP UBX GFG-RATE */
    /* TODO: init struct via pointers */
    rate_cfg.measRate = 200; /* 200 ms for measurement @5 Hz measurement rate */
    rate_cfg.navRate = 1;    /* default navigation rate (cannot be changed) */
    rate_cfg.timeRef = 0;    /* UTC reference time */

    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x08, (uint8_t*)&rate_cfg, sizeof(rate_cfg));

    /* SETUP UBX GFG-NAV5 */        /* TODO: init struct via pointers */
    nav5_cfg.dynModel = 0x07;
    nav5_cfg.mask = NAV5_dyn;

    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x24, (uint8_t*)&nav5_cfg, sizeof(nav5_cfg));

    /* SETUP BINARY MODE OUTPUT RATES */

    msg[0] = 0x01;        /* class */
    msg[1] = 0x04;        /* msg id  = UBX_NAV_DOP */
    msg[2] = 0x01;        /* rate */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 3);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x06;        /* msg id  = NAV_SOL */
    msg[2] = 0x01;        /* rate */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 3);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x02;        /* msg id  = UBX_NAV_POSLLH */
    msg[2] = 0x01;        /* rate */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 3);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x07;        /* msg id  = UBX_NAV_PVT */
    msg[2] = 0x01;        /* rate */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 3);

    msg[0] = 0x01;        /* class */
    msg[1] = 0x30;        /* msg id  = NAV-SVINFO */
    msg[2] = 0x0a;        /* rate */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x01, msg, 3);

    /* SETUP UBX CONFIG */

    /* Set USART as main protocol */
    buf[0] = USART1_ID;

    /* Set baudrate to config buffer */
    putle32(buf, 8, baudrate);

    /* Mode word setup */
    usart_mode |= (1<<4);          /* reserved1 Antaris 4 compatibility bit */
    usart_mode |= (3<<6);          /* set char_Len to 8 bits */
    usart_mode |= (parity << 9);   /* setup parity bits */
    usart_mode |= (stopbits << 9); /* setup stop bits */

    /* Setup UART mode */
    if(mode == 0){ /* If external mode config isn't set */
        putle32(buf, 4, usart_mode);
    }
    else{ /* If external mode config is set */
        putle32(buf, 4, mode);
    }

    /* Setup inProtoMask (set all input protocols as active) */
    buf[12] = NMEA_PROTOCOL_MASK | UBX_PROTOCOL_MASK | RTCM_PROTOCOL_MASK;

    /* Setup outProtoMask (set only UBX as active output protocol) */
    buf[14] = UBX_PROTOCOL_MASK;

    /* Send setup CMD */
    ubx_write(&GNSS->t_PARSE.ubx_out_buf[0], 0x06, 0x00, buf, 20);

}

/**
 * @brief   Get and parse NAV_DOP message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @param[out] status     (status == 0 => ERROR; status == 1 => OK)
 */
uint8_t ubx_msg_nav_dop(t_GNSS *GNSS,
                        uint8_t *buf,
                        size_t data_len)
{
    if (data_len != 18)
            return 0; /* Error */

        GNSS->t_DOP.gdop = (double)(getleu16(buf, 4) / 100.0);
        GNSS->t_DOP.pdop = (double)(getleu16(buf, 6) / 100.0);
        GNSS->t_DOP.tdop = (double)(getleu16(buf, 8) / 100.0);
        GNSS->t_DOP.vdop = (double)(getleu16(buf, 10) / 100.0);
        GNSS->t_DOP.hdop = (double)(getleu16(buf, 12) / 100.0);

        /* TODO: SETUP BIT FLAGS FOR NEW DATA */
        return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_SOL message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @param[out] status     (status == 0 => ERROR; status == 1 => OK)
 */
uint8_t ubx_msg_nav_sol(t_GNSS *GNSS,
                        uint8_t *buf,
                            size_t data_len)
{
    uint8_t navmode;
    uint16_t gw;
    uint32_t tow;

    if (data_len != 52)
        return 0;

        /* Get FIX information */
        navmode = (uint8_t)getub(buf, 10); /* get NAV FIX info data */
        navmode &= 0x3;    /* release only 2 LSBs */

        GNSS->t_FIX.fix = navmode;

        /* If TIME data is aviable */
        if(navmode != UBX_MODE_NOFIX)
        {
            tow = (uint32_t)getleu32(buf, 0);
            gw =  (uint16_t)getles16(buf, 8);

                GNSS->t_FIX.UTCtime = gnss_gpstime_resolve(gw, tow);
        }

    /* No of used satellites in solution */
    GNSS->t_FIX.satqty = (uint8_t)getub(buf, 47);

        /* TODO: SETUP BIT FLAGS FOR NEW DATA */
        return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_POSLLH message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @param[out] status     (status == 0 => ERROR; status == 1 => OK)
 */
uint8_t ubx_msg_nav_posllh(t_GNSS *GNSS,
                           uint8_t *buf,
                           size_t data_len)
{
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    float hAcc;
    float vAcc;

    if (data_len != 28) /* Corrupted message size */
        return 0;

        /* Get Latitude and Longitude information */
    longitude = getles32(buf, 4);
    latitude = getles32(buf, 8);
    height = getles32(buf, 12);

    /* TODO: check for NAN */
    GNSS->t_FIX.longitude = ((double)longitude) / 10000000;
    GNSS->t_FIX.latitude = ((double)latitude) / 10000000;
    GNSS->t_FIX.height = ((float)height) / 1000;

    /* Get Error covariation matrix */
    hAcc = ((float)getleu32(buf, 20)) / 1000.0f;
    vAcc = ((float)getleu32(buf, 24)) / 1000.0f;

    /* TODO: check for INF */
    GNSS->t_FIX.posCM[0] = hAcc * hAcc;
    GNSS->t_FIX.posCM[4] = GNSS->t_FIX.posCM[0];
    GNSS->t_FIX.posCM[8] = vAcc * vAcc;

    /* TODO: SETUP BIT FLAGS FOR NEW DATA */
    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_PVT message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @param[out] status     (status == 0 => ERROR; status == 1 => OK)
 */
uint8_t ubx_msg_nav_pvt(t_GNSS *GNSS,
                        uint8_t *buf,
                        size_t data_len)
{
    float sAcc;
    float tAcc;

    if (data_len != 84) /* Corrupted message size */
        return 0;

    /* Get time accuracy estimation */
    tAcc = ((float)getleu32(buf, 12)) / 1000;

    /* TODO: check for INF */
    GNSS->t_FIX.terrdisp = tAcc * tAcc;

    /* Speed in NORTH-EAST-DOWN coordinate frame */
    GNSS->t_FIX.NEDspeed[0] = ((float)getles32(buf, 48)) / 1000;
    GNSS->t_FIX.NEDspeed[1] = ((float)getles32(buf, 52)) / 1000;
    GNSS->t_FIX.NEDspeed[2] = ((float)getles32(buf, 56)) / 1000;

    /* Get estimation of speed accuracy */
    sAcc = ((float)getleu32(buf, 68)) / 1000;

    /* Set speed covatioation matrix */
    /* TODO: check for INF */
    GNSS->t_FIX.speedCM[0] = sAcc * sAcc;
    GNSS->t_FIX.speedCM[4] = GNSS->t_FIX.speedCM[0];
    GNSS->t_FIX.speedCM[8] = GNSS->t_FIX.speedCM[0];

    /* TODO: SETUP BIT FLAGS FOR NEW DATA */
    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_SVINFO message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @param[out] status     (status == 0 => ERROR; status == 1 => OK)
 */
uint8_t ubx_msg_nav_svinfo(t_GNSS *GNSS,
                           uint8_t *buf,
                           size_t data_len)
{
    (void)data_len;
    uint8_t numCh;        /* Number of channels */
    uint8_t curCh;                /* Current channel */
    uint8_t curMask;            /* Mask of current channel */
    uint8_t i;            /* Counter  */

    numCh = getub(buf, 4);

    /* Clear old information about SVs */
    memset(&GNSS->t_SATS, 0x00, sizeof(t_GNSSsats));

    /* TODO: input no check (for large no of SVs */
    for(i = 0; i < numCh; i++) /* get aviability */
    {
        /* Get SV numbers and status flag */
        curCh = getub(buf, 8 + 12*i);
        curMask = getub(buf, 10 + 12*i);

        if(curCh < NO_OF_GPS_CHANS)
        {
            /* LSB => aviability of SV */
            GNSS->t_SATS.sat[curCh].satStMask = curMask;
        }
    }

    /* TODO: SETUP BIT FLAGS FOR NEW DATA */
    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Calculate time (coupled with UNIX EPOCH)
 *
 * @param[in] week           weeks number [weeks]
 * @param[in] tow          time of week [s]
 * @param[out]             timestamp (UNIX EPOCH)
 */
uint64_t gnss_gpstime_resolve(uint16_t week, uint32_t tow)
{
    uint64_t timestamp;

    //timestamp as EPOCH + WEEK * SECS_PER_WEEK + TIME_OF_WEEK
    timestamp = (uint64_t)GPS_EPOCH + (week * SECS_PER_WEEK) + tow;
    return timestamp;
}

/** @} */
