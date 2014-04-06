/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 */

#ifndef _ublox_h
#define _ublox_h

#include "ch.h"
#include "hal.h"

#if __cplusplus
extern "C" {
#endif

/**
 * @name    ublox.h header files
 * @{
 */
#define UBX_IN_BUF_SIZE        512
#define UBX_OUT_BUF_SIZE    64
#define NO_OF_GPS_CHANS        128

#define GNSSFIX_NOFIX    0        /**< @brief GNSS FIX MODE: no fix.          */
#define GNSSFIX_TIME    1        /**< @brief GNSS FIX MODE: time only.       */
#define GNSSFIX_2D        2        /**< @brief GNSS FIX MODE: 2D.              */
#define GNSSFIX_3D        3       /**< @brief GNSS FIX MODE: 3D.              */

/*
 *        UBX MACROSES AND DATA
 */

#define UBX_MESSAGE_BASE_SIZE                 6
#define UBX_MESSAGE_DATA_OFFSET             UBX_MESSAGE_BASE_SIZE

typedef enum {
    UBX_CLASS_NAV = 0x01,     /**< Navigation */
    UBX_CLASS_RXM = 0x02,     /**< Receiver Manager */
    UBX_CLASS_INF = 0x04,     /**< Informative text messages */
    UBX_CLASS_ACK = 0x05,     /**< (Not) Acknowledges for cfg messages */
    UBX_CLASS_CFG = 0x06,     /**< Configuration requests */
    UBX_CLASS_UPD = 0x09,     /**< Firmware updates */
    UBX_CLASS_MON = 0x0a,     /**< System monitoring */
    UBX_CLASS_AID = 0x0b,     /**< AGPS */
    UBX_CLASS_TIM = 0x0d,     /**< Time */
} ubx_classes_t;

#define UBX_MSGID(cls_, id_) (((cls_)<<8)|(id_))

typedef enum {
    UBX_NAV_POSECEF    = UBX_MSGID(UBX_CLASS_NAV, 0x01),
    UBX_NAV_POSLLH    = UBX_MSGID(UBX_CLASS_NAV, 0x02),
    UBX_NAV_STATUS    = UBX_MSGID(UBX_CLASS_NAV, 0x03),
    UBX_NAV_DOP     = UBX_MSGID(UBX_CLASS_NAV, 0x04),
    UBX_NAV_SOL     = UBX_MSGID(UBX_CLASS_NAV, 0x06),
    UBX_NAV_PVT        = UBX_MSGID(UBX_CLASS_NAV, 0x07),
    UBX_NAV_POSUTM    = UBX_MSGID(UBX_CLASS_NAV, 0x08),
    UBX_NAV_VELECEF    = UBX_MSGID(UBX_CLASS_NAV, 0x11),
    UBX_NAV_VELNED    = UBX_MSGID(UBX_CLASS_NAV, 0x12),
    UBX_NAV_TIMEGPS    = UBX_MSGID(UBX_CLASS_NAV, 0x20),
    UBX_NAV_TIMEUTC    = UBX_MSGID(UBX_CLASS_NAV, 0x21),
    UBX_NAV_CLOCK   = UBX_MSGID(UBX_CLASS_NAV, 0x22),
    UBX_NAV_SVINFO    = UBX_MSGID(UBX_CLASS_NAV, 0x30),
    UBX_NAV_DGPS    = UBX_MSGID(UBX_CLASS_NAV, 0x31),
    UBX_NAV_SBAS    = UBX_MSGID(UBX_CLASS_NAV, 0x32),
    UBX_NAV_EKFSTATUS = UBX_MSGID(UBX_CLASS_NAV, 0x40),

    UBX_RXM_RAW        = UBX_MSGID(UBX_CLASS_RXM, 0x10),
    UBX_RXM_SFRB    = UBX_MSGID(UBX_CLASS_RXM, 0x11),
    UBX_RXM_SVSI    = UBX_MSGID(UBX_CLASS_RXM, 0x20),
    UBX_RXM_ALM        = UBX_MSGID(UBX_CLASS_RXM, 0x30),
    UBX_RXM_EPH        = UBX_MSGID(UBX_CLASS_RXM, 0x31),
    UBX_RXM_POSREQ    = UBX_MSGID(UBX_CLASS_RXM, 0x40),

    UBX_INF_ERROR    = UBX_MSGID(UBX_CLASS_INF, 0x00),
    UBX_INF_WARNING    = UBX_MSGID(UBX_CLASS_INF, 0x01),
    UBX_INF_NOTICE    = UBX_MSGID(UBX_CLASS_INF, 0x02),
    UBX_INF_TEST    = UBX_MSGID(UBX_CLASS_INF, 0x03),
    UBX_INF_DEBUG    = UBX_MSGID(UBX_CLASS_INF, 0x04),
    UBX_INF_USER    = UBX_MSGID(UBX_CLASS_INF, 0x07),

    UBX_ACK_NAK        = UBX_MSGID(UBX_CLASS_ACK, 0x00),
    UBX_ACK_ACK        = UBX_MSGID(UBX_CLASS_ACK, 0x01),

    UBX_CFG_PRT        = UBX_MSGID(UBX_CLASS_CFG, 0x00),

    UBX_UPD_DOWNL    = UBX_MSGID(UBX_CLASS_UPD, 0x01),
    UBX_UPD_UPLOAD    = UBX_MSGID(UBX_CLASS_UPD, 0x02),
    UBX_UPD_EXEC    = UBX_MSGID(UBX_CLASS_UPD, 0x03),
    UBX_UPD_MEMCPY    = UBX_MSGID(UBX_CLASS_UPD, 0x04),

    UBX_MON_SCHED    = UBX_MSGID(UBX_CLASS_MON, 0x01),
    UBX_MON_IO        = UBX_MSGID(UBX_CLASS_MON, 0x02),
    UBX_MON_IPC        = UBX_MSGID(UBX_CLASS_MON, 0x03),
    UBX_MON_VER        = UBX_MSGID(UBX_CLASS_MON, 0x04),
    UBX_MON_EXCEPT    = UBX_MSGID(UBX_CLASS_MON, 0x05),
    UBX_MON_MSGPP    = UBX_MSGID(UBX_CLASS_MON, 0x06),
    UBX_MON_RXBUF    = UBX_MSGID(UBX_CLASS_MON, 0x07),
    UBX_MON_TXBUF    = UBX_MSGID(UBX_CLASS_MON, 0x08),
    UBX_MON_HW        = UBX_MSGID(UBX_CLASS_MON, 0x09),
    UBX_MON_USB        = UBX_MSGID(UBX_CLASS_MON, 0x0a),

    UBX_AID_REQ        = UBX_MSGID(UBX_CLASS_AID, 0x00),
    UBX_AID_INI        = UBX_MSGID(UBX_CLASS_AID, 0x01),
    UBX_AID_HUI        = UBX_MSGID(UBX_CLASS_AID, 0x02),
    UBX_AID_DATA    = UBX_MSGID(UBX_CLASS_AID, 0x10),
    UBX_AID_ALM        = UBX_MSGID(UBX_CLASS_AID, 0x30),
    UBX_AID_EPH        = UBX_MSGID(UBX_CLASS_AID, 0x31),

    UBX_TIM_TP        = UBX_MSGID(UBX_CLASS_TIM, 0x01),
    UBX_TIM_TM        = UBX_MSGID(UBX_CLASS_TIM, 0x02),
    UBX_TIM_TM2        = UBX_MSGID(UBX_CLASS_TIM, 0x03),
    UBX_TIM_SVIN    = UBX_MSGID(UBX_CLASS_TIM, 0x04),
} ubx_message_t; /* UBX Message classes and IDs */

typedef enum {
    UBX_MODE_NOFIX  = 0x00,    /* no fix available */
    UBX_MODE_DR        = 0x01,    /* Dead reckoning */
    UBX_MODE_2D        = 0x02,    /* 2D fix */
    UBX_MODE_3D        = 0x03,    /* 3D fix */
    UBX_MODE_GPSDR  = 0x04,    /* GPS + dead reckoning */
    UBX_MODE_TMONLY = 0x05,    /* Time-only fix */
} ubx_mode_t;

typedef enum {
    UBX_PARITY_EVEN = 0x00,    /* Even Parity */
    UBX_PARITY_ODD  = 0x01,    /* Odd Parity */
    UBX_PARITY_NO   = 0x04,    /* No Parity */
} ubx7_parity_t;

typedef enum {
    UBX_STOP_ONE    = 0x00,    /* 1 Stop Bit */
    UBX_STOP_ONEH   = 0x01,    /* 1.5 Stop Bit */
    UBX_STOP_TWO    = 0x02,    /* 2 Stop Bit */
    UBX_STOP_TWOH   = 0x03,    /* No Parity */
} ubx7_stopbits_t;

#define GPS_EPOCH               315964800
#define SECS_PER_WEEK           604800

#define UBX_SOL_FLAG_GPS_FIX_OK 0x01
#define UBX_SOL_FLAG_DGPS       0x02
#define UBX_SOL_VALID_WEEK      0x04
#define UBX_SOL_VALID_TIME      0x08

/* from UBX_NAV_SVINFO */
#define UBX_SAT_USED            0x01
#define UBX_SAT_DGPS            0x02
#define UBX_SAT_EPHALM          0x04
#define UBX_SAT_EPHEM           0x08
#define UBX_SAT_UNHEALTHY       0x10

#define UBX_SIG_IDLE            0
#define UBX_SIG_SRCH1           1
#define UBX_SIG_SRCH2           2
#define UBX_SIG_DETECT          3
#define UBX_SIG_CDLK            4
#define UBX_SIG_CDCRLK1         5
#define UBX_SIG_CDCRLK2         6
#define UBX_SIG_NAVMSG          7

/*
 *            UBLOX DEFINES
 */

#define UBX_PREFIX_LEN            6
#define UBX_CLASS_OFFSET        2
#define UBX_TYPE_OFFSET         3
#define USART1_ID                1
#define USART2_ID               2
#define USB_ID                  3
#define UBX_PROTOCOL_MASK        0x01
#define NMEA_PROTOCOL_MASK      0x02
#define RTCM_PROTOCOL_MASK      0x04
#define UBX_CFG_LEN                36

/*
 *      AUX MACROSES TO GET DATA (i8, u8, i16, u16, i32, u32, f32, f64)
 */

#define     GET_ORIGIN   0
#define     PUT_ORIGIN   0
#define     getsb(buf, off)               ((int8_t)buf[(off)-(GET_ORIGIN)])
#define     getub(buf, off)               ((uint8_t)buf[(off)-(GET_ORIGIN)])
#define     putbyte(buf, off, b)           do {buf[(off)-(PUT_ORIGIN)] = (unsigned char)(b);} while (0)
#define     getles16(buf, off)           ((int16_t)(((uint16_t)getub((buf), (off)+1) << 8) | (uint16_t)getub((buf), (off))))
#define     getleu16(buf, off)           ((uint16_t)(((uint16_t)getub((buf), (off)+1) << 8) | (uint16_t)getub((buf), (off))))
#define     getles32(buf, off)           ((int32_t)(((uint16_t)getleu16((buf), (off)+2) << 16) | (uint16_t)getleu16((buf), (off))))
#define     getleu32(buf, off)           ((uint32_t)(((uint16_t)getleu16((buf),(off)+2) << 16) | (uint16_t)getleu16((buf), (off))))
#define     putle16(buf, off, w)         do {putbyte(buf, (off)+1, (uint16_t)(w) >> 8); putbyte(buf, (off), (w));} while (0)
#define     putle32(buf, off, l)         do {putle16(buf, (off)+2, (uint32_t)(l) >> 16); putle16(buf, (off), (l));} while (0)
#define     getles64(buf, off)           ((int64_t)(((uint64_t)getleu32(buf, (off)+4) << 32) | getleu32(buf, (off))))
#define     getleu64(buf, off)           ((uint64_t)(((uint64_t)getleu32(buf, (off)+4) << 32) | getleu32(buf, (off))))
#define     getlef(buf, off)                (i_f.i = getles32(buf, off), i_f.f)
#define     getled(buf, off)                (l_d.l = getles64(buf, off), l_d.d)
#define     getbes16(buf, off)           ((int16_t)(((uint16_t)getub(buf, (off)) << 8) | (uint16_t)getub(buf, (off)+1)))
#define     getbeu16(buf, off)           ((uint16_t)(((uint16_t)getub(buf, (off)) << 8) | (uint16_t)getub(buf, (off)+1)))
#define     getbes32(buf, off)           ((int32_t)(((uint16_t)getbeu16(buf, (off)) << 16) | getbeu16(buf, (off)+2)))
#define     getbeu32(buf, off)           ((uint32_t)(((uint16_t)getbeu16(buf, (off)) << 16) | getbeu16(buf, (off)+2)))
#define     getbes64(buf, off)           ((int64_t)(((uint64_t)getbeu32(buf, (off)) << 32) | getbeu32(buf, (off)+4)))
#define     getbeu64(buf, off)           ((uint64_t)(((uint64_t)getbeu32(buf, (off)) << 32) | getbeu32(buf, (off)+4)))
#define     putbe16(buf, off, w)        do {putbyte(buf, (off), (w) >> 8); putbyte(buf, (off)+1, (w));} while (0)
#define     putbe32(buf, off, l)        do {putbe16(buf, (off), (l) >> 16); putbe16(buf, (off)+2, (l));} while (0)
#define     getbef(buf, off)               (i_f.i = getbes32(buf, off), i_f.f)
#define     getbed(buf, off)               (l_d.l = getbes64(buf, off), l_d.d)


/* Pragma align to ONE byte */
#pragma pack(push, 1)


/**
 * @brief   Main navigation parameter structure.
 * @note    none
 */
typedef struct t_GNSSfix
{
    uint8_t  fix;           /**< @brief Fix mode.                       */
    double   latitude;        /**< @brief GNSS estimated latitude.        */
    double   longitude;     /**< @brief GNSS estimated longitude.       */
    float    height;        /**< @brief GNSS estimated height.          */
    float    NEDspeed[3];   /**< @brief North-East-Down velocity        */
    float      posCM[9];        /**< @brief Cov matrix of position          */
    float      speedCM[9];    /**< @brief Cov matrix of speed             */
    uint64_t UTCtime;       /**< @brief UTC time (UNIX epoch)           */
    float    terrdisp;        /**< @brief Time error dispersion           */
    uint8_t  satqty;        /**< @brief Qty of sats in NAV solution     */
}t_GNSSfix;

/**
 * @brief   DOP (degree of precision) parameters structue.
 * @note    none
 */
typedef struct t_GNSSdop
{
    float  gdop;            /**< @brief GDOP value.                     */
    float  pdop;            /**< @brief PDOP value.                     */
    float  hdop;            /**< @brief HDOP value.                     */
    float  vdop;            /**< @brief VDOP value.                     */
    float  tdop;            /**< @brief TDOP value.                     */
}t_GNSSdop;

/**
 * @brief   Satellite status information.
 * @note    none
 */
typedef struct t_GNSSsat
{
    uint8_t  satStMask;        /**< @brief satellite status mask.           */
}t_GNSSsat;

/**
 * @brief   Current information about satellites.
 * @note    none
 */
typedef struct t_GNSSsats
{
    t_GNSSsat  sat[NO_OF_GPS_CHANS];        /**< @brief complete info about satellites.   */
}t_GNSSsats;

/**
 * @brief   UBX parser helper struct
 * @note    none
 */
typedef struct t_UBXparse
{
    uint8_t ubx_in_buf[UBX_IN_BUF_SIZE];    /* input buffer for UBX data */
    uint8_t ubx_out_buf[UBX_OUT_BUF_SIZE];  /* output buffer for UBX commands */
    uint8_t ubx_parse_stat;                    /* UBX parsing status */
    /*int16_t ubx_inbuf_bp_ind;*/            /* UBX input buffer "begin-of-packet" index */
    int16_t ubx_wi;                            /* UBX input buffer write index*/
}t_UBXparse;

/**
 * @brief   Generic GNSS struct (output structs and buffer control)
 * @note    none
 */
typedef struct  t_GNSS
{
    t_UBXparse t_PARSE;                        /* PARSE structure */
    t_GNSSfix  t_FIX;                        /* FIX structure */
    t_GNSSdop  t_DOP;                        /* DOP structure */
    t_GNSSsats t_SATS;                        /* SATS structure */
}t_GNSS;

/**
 * @brief   GFG-NAV5 struct
 * @note    none
 */

typedef enum {
    NAV5_dyn        = 1 << 0,    /* Apply dynamic model settings */
    NAV5_minEl      = 1 << 1,    /* Apply minimum elevation settings */
    NAV5_posFixMode = 1 << 2,    /* Apply fix mode settings */
    NAV5_drLim      = 1 << 3,    /* Reserved */
    NAV5_dposMask   = 1 << 4,    /* Apply position mask settings */
    NAV5_timeMask   = 1 << 5,    /* Apply time mask settings */
    NAV5_sHoldMask  = 1 << 6,    /* Apply static hold settings */
    NAV5_dgpsMask   = 1 << 7,    /* Apply DGPS settings */
} CFG_NAV5_mask;

/**
 * @brief   GFG-NAV5 struct
 * @note    none
 */
typedef struct CFG_NAV5
{
    uint16_t mask;              /* Parameters bitmask */
    uint8_t  dynModel;          /* Dynamic model */
    uint8_t  fixMode;           /* Position Fixing Mode */
    int32_t  fixedAlt;          /* Fixed altitude (mean sea level) for 2D fix mode. */
    uint32_t fixedAltVar;       /* Fixed altitude variance for 2D mode. */
    int8_t   minElev;           /* Minimum Elevation for a GNSS satellite to be used in NAV */
    uint8_t  drLimit;           /* Reserved */
    uint16_t pDop;                /* Position DOP Mask to use */
    uint16_t tDop;              /* Time DOP Mask to use */
    uint16_t pAcc;              /* Position Accuracy Mask */
    uint16_t tAcc;                /* Time Accuracy Mask */
    uint8_t  staticHoldThresh;  /* Static hold threshold */
    uint8_t  dgpsTimeOut;       /* DGPS timeout */
    uint8_t  cnoThreshNumSVs;    /* Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted */
    uint8_t  cnoThresh;            /* C/N0 threshold for deciding whether to attempt a fix */
    uint16_t reserved2;            /* Reserved */
    uint32_t reserved3;            /* Reserved */
    uint32_t reserved4;            /* Reserved */
}CFG_NAV5;

/**
 * @brief   GFG-RATE struct
 * @note    none
 */

typedef struct CFG_RATE
{
    uint16_t measRate;           /* Parameters bitmask */
    uint16_t navRate;            /* Dynamic model */
    uint16_t timeRef;            /* Position Fixing Mode */
}CFG_RATE;


#pragma pack(pop)


/* MISC FUNCTIONS */
uint16_t ubx_write(uint8_t  *msgbuf, uint8_t  msg_class, uint8_t  msg_id, uint8_t *msg, uint16_t data_len);
uint64_t gnss_gpstime_resolve(uint16_t week, uint32_t tow);
int16_t ubx_findheader(uint8_t *buf, uint16_t len, uint16_t st_pos);
int16_t ubx_parse(t_GNSS *GNSS_t, uint8_t *buf, uint16_t len);
void ubx_init(t_GNSS *GNSS, uint32_t baudrate, uint8_t parity, uint8_t stopbits, uint32_t mode);
void ubx_setMSGtypes(t_GNSS *GNSS);

/* PACKET TYPE PARSING */
uint8_t ubx_msg_nav_dop(t_GNSS *GNSS, uint8_t *buf, size_t data_len);
uint8_t ubx_msg_nav_sol(t_GNSS *GNSS, uint8_t *buf, size_t data_len);
uint8_t ubx_msg_nav_posllh(t_GNSS *GNSS, uint8_t *buf, size_t data_len);
uint8_t ubx_msg_nav_pvt(t_GNSS *GNSS, uint8_t *buf, size_t data_len);
uint8_t ubx_msg_nav_svinfo(t_GNSS *GNSS, uint8_t *buf, size_t data_len);

#if __cplusplus
}
#endif

#endif /* _ublox_h */

/** @} */
