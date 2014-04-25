/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 *         Pavel Kirienko <pavel.kirienko@courierdrone.com>
 *
 * Fucking magic. Ugh.
 * FIXME rewrite from scratch
 */

#include "ublox.h"
#include <assert.h>
#include <math.h>
#include <string.h>

/*
 *      Data marshaling helpers (i8, u8, i16, u16, i32, u32, f32, f64)
 */
#define     GET_ORIGIN   0
#define     PUT_ORIGIN   0

#define     getsb(buf, off)      ((int8_t)buf[(off)-(GET_ORIGIN)])
#define     getub(buf, off)      ((uint8_t)buf[(off)-(GET_ORIGIN)])
#define     putbyte(buf, off, b) do {buf[(off)-(PUT_ORIGIN)] = (unsigned char)(b);} while (0)
#define     getles16(buf, off)   ((int16_t)(((uint16_t)getub((buf), (off)+1) << 8) | (uint16_t)getub((buf), (off))))
#define     getleu16(buf, off)   ((uint16_t)(((uint16_t)getub((buf), (off)+1) << 8) | (uint16_t)getub((buf), (off))))
#define     getles32(buf, off)   ((int32_t)(((uint16_t)getleu16((buf), (off)+2) << 16) | (uint16_t)getleu16((buf), (off))))
#define     getleu32(buf, off)   ((uint32_t)(((uint16_t)getleu16((buf),(off)+2) << 16) | (uint16_t)getleu16((buf), (off))))
#define     putle16(buf, off, w) do {putbyte(buf, (off)+1, (uint16_t)(w) >> 8); putbyte(buf, (off), (w));} while (0)
#define     putle32(buf, off, l) do {putle16(buf, (off)+2, (uint32_t)(l) >> 16); putle16(buf, (off), (l));} while (0)
#define     getles64(buf, off)   ((int64_t)(((uint64_t)getleu32(buf, (off)+4) << 32) | getleu32(buf, (off))))
#define     getleu64(buf, off)   ((uint64_t)(((uint64_t)getleu32(buf, (off)+4) << 32) | getleu32(buf, (off))))
#define     getlef(buf, off)     (i_f.i = getles32(buf, off), i_f.f)
#define     getled(buf, off)     (l_d.l = getles64(buf, off), l_d.d)
#define     getbes16(buf, off)   ((int16_t)(((uint16_t)getub(buf, (off)) << 8) | (uint16_t)getub(buf, (off)+1)))
#define     getbeu16(buf, off)   ((uint16_t)(((uint16_t)getub(buf, (off)) << 8) | (uint16_t)getub(buf, (off)+1)))
#define     getbes32(buf, off)   ((int32_t)(((uint16_t)getbeu16(buf, (off)) << 16) | getbeu16(buf, (off)+2)))
#define     getbeu32(buf, off)   ((uint32_t)(((uint16_t)getbeu16(buf, (off)) << 16) | getbeu16(buf, (off)+2)))
#define     getbes64(buf, off)   ((int64_t)(((uint64_t)getbeu32(buf, (off)) << 32) | getbeu32(buf, (off)+4)))
#define     getbeu64(buf, off)   ((uint64_t)(((uint64_t)getbeu32(buf, (off)) << 32) | getbeu32(buf, (off)+4)))
#define     putbe16(buf, off, w) do {putbyte(buf, (off), (w) >> 8); putbyte(buf, (off)+1, (w));} while (0)
#define     putbe32(buf, off, l) do {putbe16(buf, (off), (l) >> 16); putbe16(buf, (off)+2, (l));} while (0)
#define     getbef(buf, off)     (i_f.i = getbes32(buf, off), i_f.f)
#define     getbed(buf, off)     (l_d.l = getbes64(buf, off), l_d.d)

/*
 * Local functions
 */
static int16_t ubxFindheader(uint8_t *buf, uint16_t len, uint16_t stpos);
static int16_t ubxParse(UbxState *ubx, uint8_t *buf, uint16_t len);
static void fixStatDataUpdate(UbxState *ubx, uint8_t fixstatus);

static void ubxSetMSGtypes(void);

/* PACKET TYPE PARSING */
static uint8_t ubxMsgNavTimeGps(UbxState *ubx, uint8_t *buf, size_t len);
static uint8_t ubxMsgNavDop(UbxState *ubx, uint8_t *buf, size_t len);
static uint8_t ubxNsgNavSol(UbxState *ubx, uint8_t *buf, size_t len);
static uint8_t ubxMsgNavPosllh(UbxState *ubx, uint8_t *buf, size_t len);
static uint8_t ubxMsgNavPvt(UbxState *ubx, uint8_t *buf, size_t len);
static uint8_t ubxMsgNavSvinfo(UbxState *ubx, uint8_t *buf, size_t len);

/**
 * @brief   Form UBX message to uint8_t array
 *
 * @param[in] msg_class   UBX message class
 * @param[in] msg_id      UBX message ID
 * @param[in] msg         pointer to UBX message body
 * @param[in] data_len    lenght of message body
 * @return                total number of bytes in output array
 */
uint16_t ubxWrite(uint8_t msg_class, uint8_t msg_id, uint8_t *msg, uint16_t data_len)
{
    uint8_t msgbuf[256];
    uint8_t CK_A, CK_B;
    uint16_t i;

    memset(&msgbuf, 0, sizeof(msgbuf));

    /*@ -type @*/
    msgbuf[0] = 0xb5;
    msgbuf[1] = 0x62;

    CK_A = CK_B = 0;
    msgbuf[2] = msg_class;
    msgbuf[3] = msg_id;
    msgbuf[4] = data_len & 0xff;
    msgbuf[5] = (data_len >> 8) & 0xff;

    /* check for presence of data */
    if (msg != NULL)
    {
        memcpy(&msgbuf[6], msg, data_len);
    }

    /* calculate CRC */
    for (i = 2; i < 6; i++)
    {
        CK_A += msgbuf[i];
        CK_B += CK_A;
    }

    /*@ -nullderef @*/
    if (msg != NULL)
    {
        for (i = 0; i < data_len; i++)
        {
            CK_A += msg[i];
            CK_B += CK_A;
        }
    }

    msgbuf[6 + data_len] = CK_A;
    msgbuf[7 + data_len] = CK_B;

    (void)sdWrite(&SD2, msgbuf, data_len + 8);
    return (data_len + 8);
}

/**
 * @brief   Find UBX message header in uint8_t array
 *
 * @param[in] *buf        pointer to buffer (to find UBX header)
 * @param[in] len         length of input buffer
 * @param[in] stpos       start position of search
 * @return                position of the first UBX message in the buffer (index == -1 if the header is not found)
 */
static int16_t ubxFindheader(uint8_t *buf, uint16_t len, uint16_t stpos)
{
    uint16_t i; /* counter variable */
    int16_t hind; /* position of first header index */

    hind = -1;

    /* Try to find UBX header */
    for (i = stpos; i < len - stpos - 1; i++)
    {
        if (buf[i] == 0xB5)
        {
            if (buf[i + 1] == 0x62)
            {
                hind = i;
                break;
            }
        }
    }

    /* Return first index number */
    return hind;
}

/**
 * @brief   Calculate UBX CRC in uint8_t array
 *
 * @param[in] *buf        pointer to buffer (to calculate UBX CRC)
 * @param[in] len         length of input buffer
 * @return                CRC
 */
static uint16_t ubxCalcCRC(uint8_t *buf, uint16_t len)
{
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    uint16_t i;
    uint16_t crc;

    /* Calculate CRC */
    for (i = 0; i < len; i++)
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
 * @param[in] *UbxState     pointer to GNSS structure (to update GNSS parameters)
 * @param[in] *buf        pointer to buffer (to parse UBX packet)
 * @param[in] len         length of input buffer
 * @return                size of parsed UBX message (negative = UBX parsing error)
 */
static int16_t ubxParse(UbxState *ubx, uint8_t *Buf, uint16_t Len)
{
    uint16_t data_len; /* Total lenght of data */
    uint16_t msg_id; /* Message ID */
    uint16_t CRCp; /* CRC in packet */
    uint16_t CRCc; /* CRC calculated */

    /* The packet at least contains a head long enough for an empty message */
    if (Len < Ubx_PrefixLen)
        return -1; /* Parse ERROR: not sufficient data */

    /* Extract message id and length */
    msg_id = (Buf[2] << 8) | Buf[3];
    data_len = (size_t) getles16(Buf, 4);

    if (Len < data_len + 8)
        return -1; /* Parse ERROR: not sufficient data */

    CRCp = (Buf[data_len + 6] << 8) | Buf[data_len + 7]; /* Get  CRC from packet */
    CRCc = ubxCalcCRC(&Buf[2], data_len + 4); /* Calc CRC for packet body */

    if (CRCp != CRCc)
    {
        return -2; /* CRC ERROR: calculated CRC not equal with packet CRC */
    }

    /* Decode packet & update GNSS data */
    switch (msg_id)
    {
    case UbxNavTimegps:
    {
        (void)ubxMsgNavTimeGps(ubx, &Buf[6], data_len);
        ubx->parse.packstat |= UbxNavTimeGpsUp;
        break;
    }
    case UbxNavDop: /* UBX_NAV_DOP */
    {
        if (ubxMsgNavDop(ubx, &Buf[6], data_len) != 0)
        { /* NAV DOP parsing SUCCESS */
            ubx->parse.packstat |= UbxNavDopUp;
        }
        else
        {
            /* Error in parsing NAV DOP output message */
        }
        break;
    }
    case UbxNavSol: /* UBX NAV SOL */
    {
        if (ubxNsgNavSol(ubx, &Buf[6], data_len) != 0)
        { /* NAV SOL parsing SUCCESS */
            ubx->parse.packstat |= UbxNavSolUp;
        }
        else
        {
            /* Error in parsing NAV SOL output message */
        }
        break;
    }
    case UbxNavPosllh: /* UBX NAV POSLLH */
    {
        if (ubxMsgNavPosllh(ubx, &Buf[6], data_len) != 0)
        { /* NAV POSLLH parsing SUCCESS */
            ubx->parse.packstat |= UbxNavPosllhUp;
        }
        else
        {
            /* Error in parsing NAV POSLLH output message */
        }
        break;
    }
    case UbxNavPvt: /* UBX NAV PVT */
    {
        if (ubxMsgNavPvt(ubx, &Buf[6], data_len) != 0)
        { /* NAV PVT parsing SUCCESS */
            ubx->parse.packstat |= UbxNavPvtUp;
        }
        else
        {
            /* Error in parsing NAV PVT output message */
        }
        break;
    }
    case UbxNavSvinfo: /* UBX NAV SVINFO */
    {
        if (ubxMsgNavSvinfo(ubx, &Buf[6], data_len) != 0)
        { /* NAV SVINFO parsing SUCCESS */
            ubx->parse.packstat |= UbxNavSvinfoUp;
        }
        else
        {
            /* Error in parsing NAV PVT output message */
        }
        break;
    }
    case UbxAckAck:
    {
        assert(data_len == 2);
        break;
    }
    default:
    {
        assert(0);
        break;
    }
    }
    return (data_len + 8);
}

/**
 * @brief    Set required msg types to recieve (UBX MODE)
 */
static void ubxSetMSGtypes(void) /* u-Center UBX example */
{
    uint8_t msg[4];
    /*
     * UBX_NAV_DOP
     */
    msg[0] = 0x01;
    msg[1] = 0x04;
    msg[2] = 5;
    ubxWrite(0x06, 0x01, msg, 3);
    /*
     * NAV_SOL
     */
    msg[0] = 0x01;
    msg[1] = 0x06;
    msg[2] = 1;
    ubxWrite(0x06, 0x01, msg, 3);
    /*
     * UBX_NAV_POSLLH
     */
    msg[0] = 0x01;
    msg[1] = 0x02;
    msg[2] = 1;
    ubxWrite(0x06, 0x01, msg, 3);
    /*
     * UBX_NAV_PVT
     */
    msg[0] = 0x01;
    msg[1] = 0x07;
    msg[2] = 1;
    ubxWrite(0x06, 0x01, msg, 3);
    /*
     * NAV-SVINFO
     */
    msg[0] = 0x01;
    msg[1] = 0x30;
    msg[2] = 10;
    ubxWrite(0x06, 0x01, msg, 3);
    /*
     * UBX_NAV_DOP
     */
    msg[0] = 0x01;
    msg[1] = 0x04;
    msg[2] = 5;
    ubxWrite(0x06, 0x01, msg, 3);
    /*
     * UBX_NAV_TIMEGPS
     */
    msg[0] = 0x01;
    msg[1] = 0x20;
    msg[2] = 1;
    ubxWrite(0x06, 0x01, msg, 3);
}

/**
 * @brief   Init UBX mode with known baudrate, parity bits setup, stopbits and mode
 *
 * @param[in] baudrate    uart port baudrate setup
 */
void ubxInit(UbxState *ubx, uint32_t baudrate)
{
    /* GNSS FIX */
    ubx->fix.fix = GNSSfix_NoFix;
    ubx->fix.latitude = NAN;
    ubx->fix.longitude = NAN;
    ubx->fix.altitude = NAN;

    for (uint8_t i = 0; i < 3; i++)
    {
        ubx->fix.ned_speed[i] = NAN; /* SET TO Not-a-number */
    }

    for (uint8_t i = 0; i < 9; i++)
    {
        ubx->fix.pos_cov[i] = 0.0;
        ubx->fix.speed_cov[i] = 0.0;
    }

    ubx->fix.sats_used = 0;

    /* GNSS DOP */
    ubx->dop.gdop = INFINITY;
    ubx->dop.pdop = INFINITY;
    ubx->dop.hdop = INFINITY;
    ubx->dop.vdop = INFINITY;
    ubx->dop.tdop = INFINITY;

    /* GNSS SATS */
    memset(&ubx->sats, 0, sizeof(ubx->sats));

    /* === COMM INTERFACE SETUP === */
    /* Messages and transmitting modes */

    /*
     * CFG-RATE
     */
    CfgRate RateCfg __attribute__((aligned(8)));
    assert(sizeof(RateCfg) == 6);
    memset(&RateCfg, 0, sizeof(RateCfg));

    RateCfg.meas_rate = 200; /* 200 ms for measurement @5 Hz measurement rate */
    RateCfg.nav_rate = 1;    /* default navigation rate (cannot be changed) */
    RateCfg.time_ref = 0;    /* align to UTC reference time */

    ubxWrite(0x06, 0x08, (uint8_t*)&RateCfg, sizeof(RateCfg));

    /*
     * GFG-NAV5
     */
    CfgNav5 Nav5Cfg __attribute__((aligned(8)));
    assert(sizeof(Nav5Cfg) == 36);
    memset(&Nav5Cfg, 0, sizeof(Nav5Cfg));

    Nav5Cfg.mask = NAV5_dyn;
    Nav5Cfg.dyn_model = 7;         // Airborne 2g

    ubxWrite(0x06, 0x24, (uint8_t*)&Nav5Cfg, sizeof(Nav5Cfg));

    /*
     * Configure msg rates
     */
    ubxSetMSGtypes();

    /*
     * CFG-PRT
     */
    CfgPrt PrtCfg __attribute__((aligned(8)));
    assert(sizeof(PrtCfg) == 20);
    memset(&PrtCfg, 0, sizeof(PrtCfg));

    PrtCfg.portid = Ubx_Usart1_id;
    PrtCfg.mode = 0x000008D0; /* 8bit, 1stop, no parity check */
    PrtCfg.baudrate = baudrate;
    PrtCfg.inprotomask  = Ubx_PrMask_UBX | Ubx_PrMask_RTCM;
    PrtCfg.outprotomask = Ubx_PrMask_UBX;

    ubxWrite(0x06, 0x00, (uint8_t*)&PrtCfg, sizeof(PrtCfg));
}

void ubxPoll(UbxState *ubx)
{
    assert(ubx != NULL);
    assert(ubx->parse.inbuf_size >= 0 && ubx->parse.inbuf_size <= UbxInBufSize);

    const int16_t max_sz = UbxInBufSize - ubx->parse.inbuf_size;
    const int16_t sz = sdReadTimeout(&SD2, ubx->parse.inbuf + ubx->parse.inbuf_size, max_sz, MS2ST(1));
    assert(sz <= max_sz);
    if (sz <= 0)
    {
        return;
    }
    ubx->parse.inbuf_size += sz;
    assert(ubx->parse.inbuf_size >= 0 && ubx->parse.inbuf_size <= UbxInBufSize);

    while (1)
    {
        // Find the first header and discard all preceding data
        {
            const int16_t header_index = ubxFindheader(ubx->parse.inbuf, ubx->parse.inbuf_size, 0);
            if (header_index < 0)
            {
                ubx->parse.inbuf_size = 0;
                break;
            }
            assert(header_index < ubx->parse.inbuf_size);
            if (header_index > 0)
            {
                memmove(ubx->parse.inbuf, ubx->parse.inbuf + header_index, ubx->parse.inbuf_size - header_index);
                ubx->parse.inbuf_size -= header_index;
            }
        }

        // Process the message then discard its data
        {
            int16_t num_discard = ubxParse(ubx, ubx->parse.inbuf, ubx->parse.inbuf_size);
            if (num_discard < 0)
            {
                if (ubx->parse.inbuf_size == UbxInBufSize)
                {
                    /*
                     * Failed to parse AND buffer is full --> we stuck, buffer contains invalid data.
                     * Discard the header to parse the rest.
                     */
                    num_discard = 1;
                }
                else
                {
                    break;
                }
            }
            assert(num_discard <= ubx->parse.inbuf_size);
            memmove(ubx->parse.inbuf, ubx->parse.inbuf + num_discard, ubx->parse.inbuf_size - num_discard);
            ubx->parse.inbuf_size -= num_discard;
        }
    }
}

static uint8_t ubxMsgNavTimeGps(UbxState *ubx, uint8_t *buf, size_t len)
{
    if (len != 16)
        return 0;

    const int64_t gps_tow_ms = getleu32(buf, 0);
    const int64_t gps_tow_ns = getles32(buf, 4);
    const int64_t gps_week   = getles16(buf, 8);
    const int64_t leap_secs  = getsb(buf, 10);

    const uint8_t valid = getub(buf, 11);

    // Base UTC offset
    int64_t utc_usec = 315964800LL * 1000000LL;
    // Uncorrected UTC time
    utc_usec += (gps_week * SecsPerWeek * 1000000LL) + (gps_tow_ms * 1000LL) + (gps_tow_ns / 1000LL);
    // Subtract leap seconds
    utc_usec -= leap_secs * 1000000LL;

    if (utc_usec < 1398426084000000 || utc_usec > 9999999999000000)
    {
        ubx->time.utc_usec = 0;
        ubx->time.valid = false;
    }
    else
    {
        ubx->time.utc_usec = utc_usec;
        ubx->time.valid = (valid & 0b111) == 0b111;
    }

    return 1;
}

/**
 * @brief   Get and parse NAV_DOP message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @return    status      (status == 0 => ERROR; status == 1 => OK)
 */
static uint8_t ubxMsgNavDop(UbxState *ubx, uint8_t *buf, size_t len)
{
    if (len != 18)
        return 0; /* Error */

    ubx->dop.gdop = (float)(getleu16(buf, 4) / 100.0);
    ubx->dop.pdop = (float)(getleu16(buf, 6) / 100.0);
    ubx->dop.tdop = (float)(getleu16(buf, 8) / 100.0);
    ubx->dop.vdop = (float)(getleu16(buf, 10) / 100.0);
    ubx->dop.hdop = (float)(getleu16(buf, 12) / 100.0);

    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_SOL message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @return    status      (status == 0 => ERROR; status == 1 => OK)
 */
static uint8_t ubxNsgNavSol(UbxState *ubx, uint8_t *buf, size_t len)
{
    uint8_t navmode; /* Navigation mode: none, time only, 2D, 3D */

    if (len != 52)
        return 0;

    /* Get FIX information */
    navmode = (uint8_t) getub(buf, 10); /* get NAV FIX info data */
    navmode &= 0x3; /* release only 2 LSBs */

    ubx->fix.fix = (UbxFixMode) navmode;

    /* Update GNSS information due to FIX status */
    fixStatDataUpdate(ubx, navmode);

    /* No of used satellites in solution */
    ubx->fix.sats_used = (uint8_t) getub(buf, 47);

    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_POSLLH message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @return    status      (status == 0 => ERROR; status == 1 => OK)
 */
static uint8_t ubxMsgNavPosllh(UbxState *ubx, uint8_t *buf, size_t len)
{
    int32_t longitude;
    int32_t latitude;
    int32_t altitude;
    float h_acc;
    float v_acc;

    if (len != 28) /* Corrupted message size */
        return 0;

    /* Get Latitude and Longitude information */
    longitude = getles32(buf, 4);
    latitude = getles32(buf, 8);
    altitude = getles32(buf, 12);

    ubx->fix.longitude = ((double)longitude) / 1e7;
    ubx->fix.latitude = ((double)latitude)   / 1e7;
    ubx->fix.altitude = ((float)altitude) / 1e3F;

    /* Get Error covariance matrix */
    h_acc = ((float)getleu32(buf, 20)) / 1e3F;
    v_acc = ((float)getleu32(buf, 24)) / 1e3F;

    ubx->fix.pos_cov[0] = h_acc * h_acc;
    ubx->fix.pos_cov[4] = h_acc * h_acc;
    ubx->fix.pos_cov[8] = v_acc * v_acc;

    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_PVT message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @return    status      (status == 0 => ERROR; status == 1 => OK)
 */
static uint8_t ubxMsgNavPvt(UbxState *ubx, uint8_t *buf, size_t len)
{
    if (len != 84) /* Corrupted message size */
        return 0;

    /* Speed in NORTH-EAST-DOWN coordinate frame */
    ubx->fix.ned_speed[0] = ((float)getles32(buf, 48)) / 1000;
    ubx->fix.ned_speed[1] = ((float)getles32(buf, 52)) / 1000;
    ubx->fix.ned_speed[2] = ((float)getles32(buf, 56)) / 1000;

    /* Get estimation of speed accuracy */
    const float s_acc = ((float) getleu32(buf, 68)) / 1000;

    /* Set speed covariance matrix */
    ubx->fix.speed_cov[0] = s_acc * s_acc;
    ubx->fix.speed_cov[4] = s_acc * s_acc;
    ubx->fix.speed_cov[8] = s_acc * s_acc;

    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Get and parse NAV_SVINFO message
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] buf*        pointer to buffer
 * @param[in] data_len    data len of buffer
 * @return    status      (status == 0 => ERROR; status == 1 => OK)
 */
static uint8_t ubxMsgNavSvinfo(UbxState *ubx, uint8_t *buf, size_t len)
{
    uint8_t num_ch; /* Number of channels */
    uint8_t cur_ch; /* Current channel */
    uint8_t cur_mask; /* Mask of current channel */
    uint8_t i; /* Counter  */

    (void)len;

    num_ch = getub(buf, 4);

    /* Clear old information about SVs */
    memset(&ubx->sats, 0x00, sizeof(UbxSatsInfo));
    ubx->fix.sats_used = 0;

    /* TODO: save information about actial SVs only */
    for (i = 0; i < num_ch; i++) /* get aviability */
    {
        /* Get SV numbers and status flag */
        cur_ch = getub(buf, 8 + 12*i);
        cur_mask = getub(buf, 10 + 12*i);

        if (cur_ch < NoOfGNSSch)
        {
            /* LSB => aviability of SV */
            ubx->sats.sat[cur_ch].sat_stmask = cur_mask;
            if (cur_mask & UbxSVM_SvUsed)
            {
                ubx->fix.sats_used++; /* Information is used in navitagion solution */
            }
        }
    }

    return 1; /* NAV_DOP parsed OK */
}

/**
 * @brief   Update data in GNSS structure due to information about fix status
 *
 * @param[in] t_GNSS      GNSS generic struct
 * @param[in] fixStatus   Fix status information (noFix,
 */
static void fixStatDataUpdate(UbxState *ubx, uint8_t fixstatus)
{
    /* Reset 3D information */
    if ((fixstatus & 0x03) < UbxMode_3D)
    {
        ubx->fix.altitude = NAN;     /* altitude is unknown */
        ubx->fix.ned_speed[2] = NAN; /* climb rate is unknown */
    }

    /* Reset 2D information */
    if ((fixstatus & 0x03) < UbxMode_2D)
    {
        ubx->fix.latitude = NAN;
        ubx->fix.longitude = NAN;
        ubx->fix.ned_speed[0] = NAN;
        ubx->fix.ned_speed[1] = NAN;
    }
}

/**
 * @brief   Get informaton about GNSS information structures
 *
 * @param[in] t_GNSS       GNSS generic struct
 * @param[in] st_bf        structure bitfield
 * @return                 isReady ([0] = isNotReady; [1] = isReady)
 */
bool ubxGetStReadyStat(UbxState *ubx, uint16_t st_bf)
{
    uint16_t mask = 0;

    switch (st_bf)
    {
    case TimeSt:
    {
        mask = UbxNavTimeGpsUp;
        break;
    }
    case FixSt:
    {
        mask = UbxNavSolUp | UbxNavPosllhUp | UbxNavPvtUp;
        break;
    }
    case DopSt:
    {
        mask = UbxNavDopUp;
        break;
    }
    case SatSt:
    {
        mask = UbxNavSvinfoUp;
        break;
    }
    default:
    {
        assert(0);
        break;
    }
    }
    return (ubx->parse.packstat & mask) == mask;
}

/**
 * @brief   Reset informaton about GNSS information structures
 *
 * @param[in] t_GNSS       GNSS generic struct
 * @param[in] st_bf        structure bitfield
 * @return                 isReady ([0] = isNotReady; [1] = isReady)
 */
void ubxResetStReadyStat(UbxState *ubx, uint16_t st_bf)
{
    switch (st_bf)
    {
    case TimeSt:
    {
        ubx->parse.packstat &= ~UbxNavTimeGpsUp;
        break;
    }
    case FixSt:
    {
        ubx->parse.packstat &= ~(UbxNavSolUp | UbxNavPosllhUp | UbxNavPvtUp);
        break;
    }
    case DopSt:
    {
        ubx->parse.packstat &= ~(UbxNavDopUp);
        break;
    }
    case SatSt:
    {
        ubx->parse.packstat &= ~(UbxNavSvinfoUp);
        break;
    }
    default:
    {
        assert(0);
        break;
    }
    }
}
