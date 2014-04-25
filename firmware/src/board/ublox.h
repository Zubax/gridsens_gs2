/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Alexander Buraga <dtp-avb@yandex.ru>
 *         Pavel Kirienko <pavel.kirienko@courierdrone.com>
 *
 * TODO: Current implementation is a garbage. Someday this shall be burned and rewritten from scratch.
 */

#pragma once

#include <ch.h>
#include <hal.h>

#if __cplusplus
extern "C" {
#endif

#define UbxInBufSize      512        /**< @brief Temp buffer (to hold ubx message tail) size   */
#define NoOfGNSSch        128        /**< @brief Number of satellites to track   */

/*
 *        UBX MACROS AND DATA
 */

#define UbxMessageBaseSize     6
#define UbxMessageDataOffset   UbxMessageBaseSize

typedef enum
{
    UbxSVM_SvUsed = 1 << 0,     /**< @brief UBX space vehicle mask: SV is used in navigational solution */
    UbxSVM_DiffCorr = 1 << 1,   /**< @brief UBX space vehicle mask: Differentical correction is aviable  */
    UbxSVM_OrbitAvail = 1 << 2, /**< @brief UBX space vehicle mask: Orbit information is aviable (Eph or Alm) */
    UbxSVM_OrbitEph = 1 << 3,   /**< @brief UBX space vehicle mask: Orbit information if Ephemeris */
    UbxSVM_Unhealthy = 1 << 4,  /**< @brief UBX space vehicle mask: SV shall not be used */
    UbxSVM_OrbitAlm = 1 << 5,   /**< @brief UBX space vehicle mask: Orbit information is Almanac Plus */
    UbxSVM_OrbitAop = 1 << 6,   /**< @brief UBX space vehicle mask: Orbit information is AssistNow Autonomous */
    UbxSVM_Smoothed = 1 << 7    /**< @brief UBX space vehicle mask: Carrier smoothed pseudocarriage used */
} UbxSVMask;

typedef enum
{
    GNSSfix_NoFix = 0, /**< @brief GNSS FIX MODE: no fix.          */
    GNSSfix_Time = 1,  /**< @brief GNSS FIX MODE: time only.       */
    GNSSfix_2D = 2,    /**< @brief GNSS FIX MODE: 2D.              */
    GNSSfix_3D = 3     /**< @brief GNSS FIX MODE: 3D.              */
} UbxFixMode;

typedef enum
{
    UbxClassNav = 0x01, /**< Navigation */
    UbxClassRxm = 0x02, /**< Receiver Manager */
    UbxClassInf = 0x04, /**< Informative text messages */
    UbxClassAck = 0x05, /**< (Not) Acknowledges for cfg messages */
    UbxClassCfg = 0x06, /**< Configuration requests */
    UbxClassUpd = 0x09, /**< Firmware updates */
    UbxClassMon = 0x0a, /**< System monitoring */
    UbxClassAid = 0x0b, /**< AGPS */
    UbxClassTim = 0x0d  /**< Time */
} UbxClasses;

typedef enum
{
    UbxNavDopBit = 1 << 0,    /**< NAV DOP msg bitfield */
    UbxNavSolBit = 1 << 1,    /**< NAV SOL msg bitfield */
    UbxNavPosllhBit = 1 << 2, /**< NAV POSLLH msg bitfield */
    UbxNavPvtBit = 1 << 3,    /**< NAV PVT msg bitfield */
    UbxNavSvinfoBit = 1 << 4  /**< NAV SVINFO msg bitfield */
} UbxMsgBitfield;

/** Compile UBX header from class and ID */
#define UBX_MSGID(cls_, id_) (((cls_)<<8)|(id_))

enum
{
    UbxNavPosecef = UBX_MSGID(UbxClassNav, 0x01),
    UbxNavPosllh = UBX_MSGID(UbxClassNav, 0x02),
    UbxNavStatus = UBX_MSGID(UbxClassNav, 0x03),
    UbxNavDop = UBX_MSGID(UbxClassNav, 0x04),
    UbxNavSol = UBX_MSGID(UbxClassNav, 0x06),
    UbxNavPvt = UBX_MSGID(UbxClassNav, 0x07),
    UbxNavPosutm = UBX_MSGID(UbxClassNav, 0x08),
    UbxNavVelecef = UBX_MSGID(UbxClassNav, 0x11),
    UbxNavVelned = UBX_MSGID(UbxClassNav, 0x12),
    UbxNavTimegps = UBX_MSGID(UbxClassNav, 0x20),
    UbxNavTimeutc = UBX_MSGID(UbxClassNav, 0x21),
    UbxNavClock = UBX_MSGID(UbxClassNav, 0x22),
    UbxNavSvinfo = UBX_MSGID(UbxClassNav, 0x30),
    UbxNavDgps = UBX_MSGID(UbxClassNav, 0x31),
    UbxNavSbas = UBX_MSGID(UbxClassNav, 0x32),
    UbxNavEkfstatus = UBX_MSGID(UbxClassNav, 0x40),

    UbxRxmRaw = UBX_MSGID(UbxClassRxm, 0x10),
    UbxRxmSfrb = UBX_MSGID(UbxClassRxm, 0x11),
    UbxRxmSvsi = UBX_MSGID(UbxClassRxm, 0x20),
    UbxRxmAlm = UBX_MSGID(UbxClassRxm, 0x30),
    UbxRxmEph = UBX_MSGID(UbxClassRxm, 0x31),
    UbxRxmPosreq = UBX_MSGID(UbxClassRxm, 0x40),

    UbxInfError = UBX_MSGID(UbxClassInf, 0x00),
    UbxInfWarning = UBX_MSGID(UbxClassInf, 0x01),
    UbxInfNotice = UBX_MSGID(UbxClassInf, 0x02),
    UbxInfTest = UBX_MSGID(UbxClassInf, 0x03),
    UbxInfDebug = UBX_MSGID(UbxClassInf, 0x04),
    UbxInfUser = UBX_MSGID(UbxClassInf, 0x07),

    UbxAckNak = UBX_MSGID(UbxClassAck, 0x00),
    UbxAckAck = UBX_MSGID(UbxClassAck, 0x01),

    UbxCfgPrt = UBX_MSGID(UbxClassCfg, 0x00),

    UbxUpdDownl = UBX_MSGID(UbxClassUpd, 0x01),
    UbxUpdUpload = UBX_MSGID(UbxClassUpd, 0x02),
    UbxUpdExec = UBX_MSGID(UbxClassUpd, 0x03),
    UbxUpdMemcpy = UBX_MSGID(UbxClassUpd, 0x04),

    UbxMonSched = UBX_MSGID(UbxClassMon, 0x01),
    UbxMonIo = UBX_MSGID(UbxClassMon, 0x02),
    UbxMonIpc = UBX_MSGID(UbxClassMon, 0x03),
    UbxMonVer = UBX_MSGID(UbxClassMon, 0x04),
    UbxMonExcept = UBX_MSGID(UbxClassMon, 0x05),
    UbxMonMsgpp = UBX_MSGID(UbxClassMon, 0x06),
    UbxMonRxbuf = UBX_MSGID(UbxClassMon, 0x07),
    UbxMonTxbuf = UBX_MSGID(UbxClassMon, 0x08),
    UbxMonHw = UBX_MSGID(UbxClassMon, 0x09),
    UbxMonUsb = UBX_MSGID(UbxClassMon, 0x0a),

    UbxAidReq = UBX_MSGID(UbxClassAid, 0x00),
    UbxAidIni = UBX_MSGID(UbxClassAid, 0x01),
    UbxAidHui = UBX_MSGID(UbxClassAid, 0x02),
    UbxAidData = UBX_MSGID(UbxClassAid, 0x10),
    UbxAidAlm = UBX_MSGID(UbxClassAid, 0x30),
    UbxAidEph = UBX_MSGID(UbxClassAid, 0x31),

    UbxTimTp = UBX_MSGID(UbxClassTim, 0x01),
    UbxTimTm = UBX_MSGID(UbxClassTim, 0x02),
    UbxTimTm2 = UBX_MSGID(UbxClassTim, 0x03),
    UbxTimSvin = UBX_MSGID(UbxClassTim, 0x04)
};

enum
{
    UbxMode_NOFIX = 0x00,   /* no fix available */
    UbxMode_DR = 0x01,      /* Dead reckoning */
    UbxMode_2D = 0x02,      /* 2D fix */
    UbxMode_3D = 0x03,      /* 3D fix */
    UbxMode_GPSDR = 0x04,   /* GPS + dead reckoning */
    UbxMode_TIMONLY = 0x05  /* Time-only fix */
};

#define SecsPerWeek             604800

#define UbxSol_FlagGpsFixOk     0x01
#define UbxSol_FlagDgps         0x02
#define UbxSol_ValidWeek        0x04
#define UbxSol_ValidTime        0x08

/* from UBX_NAV_SVINFO */
#define UbxSat_Used             0x01
#define UbxSat_Dgps             0x02
#define UbxSat_EphAlm           0x04
#define UbxSat_Ephem            0x08
#define UbxSat_Unhealthy        0x10

#define UbxSig_Idle             0
#define UbxSig_Srch1            1
#define UbxSig_Srch2            2
#define UbxSig_Detect           3
#define UbxSig_Cdlk             4
#define UbxSig_Cdcrlk1          5
#define UbxSig_Cdcrlk2          6
#define UbxSig_NavMsg           7

/*
 *            UBLOX DEFINES
 */

#define Ubx_PrefixLen           6     /* UBX prefix lenght (UBX header + class + type + lenght) */
#define Ubx_ClassOffset         2     /* Class offset (UBX packet) */
#define Ubx_TypeOffset          3     /* Type offset (UBX packet) */
#define Ubx_Usart1_id           1     /* Interface ID: USART1 */
#define Ubx_Usart2_id           2     /* Interface ID: USART2 */
#define Ubx_USB_id              3     /* Interface ID: USB */
#define Ubx_PrMask_UBX          0x01  /* UBX protocol mask: UBX */
#define Ubx_PrMask_NMEA         0x02  /* UBX protocol mask: NMEA */
#define Ubx_PrMask_RTCM         0x04  /* UBX protocol mask: RTCM */

typedef struct
{
    UbxFixMode fix;     /**< @brief Fix mode.                       */
    double latitude;    /**< @brief GNSS estimated latitude.        */
    double longitude;   /**< @brief GNSS estimated longitude.       */
    float altitude;     /**< @brief GNSS estimated altitude.        */
    float ned_speed[3]; /**< @brief North-East-Down velocity        */
    float pos_cov[9];   /**< @brief Cov matrix of position          */
    float speed_cov[9]; /**< @brief Cov matrix of speed             */
    uint8_t sats_used;   /**< @brief Qty of sats in NAV solution     */
} UbxFix;

typedef struct
{
    uint64_t utc_usec;
    bool valid;
} UbxTime;

typedef struct
{
    float gdop; /**< @brief GDOP value.                     */
    float pdop; /**< @brief PDOP value.                     */
    float hdop; /**< @brief HDOP value.                     */
    float vdop; /**< @brief VDOP value.                     */
    float tdop; /**< @brief TDOP value.                     */
} UbxDop;

typedef struct
{
    uint8_t sat_stmask; /**< @brief satellite status mask.           */
} UbxSatInfo;

typedef struct
{
    UbxSatInfo sat[NoOfGNSSch]; /**< @brief complete info about satellites.   */
} UbxSatsInfo;

typedef struct
{
    uint8_t inbuf[UbxInBufSize]; /* input buffer for UBX data */
    uint16_t packstat;           /* UBX packet collection status:
                                  * 0b1 - ready to send (and not sent yet), 0b0 - not ready to send */
    int16_t inbuf_size;
} UbxParserState;

typedef struct
{
    UbxParserState parse;
    UbxFix fix;
    UbxDop dop;
    UbxSatsInfo sats;
    UbxTime time;
} UbxState;

enum
{
    FixSt = 1 << 0, /* FIX structure */
    DopSt = 1 << 1, /* DOP structure */
    SatSt = 1 << 2, /* SATS structure */
    TimeSt = 1 << 3
};

/**
 * @brief   Bitfields for successfully recieved and parsed packets
 * @note    none
 */
enum
{
    UbxNavDopUp = 1 << 0,    /* NavDop message is successfully updated */
    UbxNavSolUp = 1 << 1,    /* NavSol message is successfully updated */
    UbxNavPosllhUp = 1 << 2, /* NavPosllh message is successfully updated */
    UbxNavPvtUp = 1 << 3,    /* NavPvt message is successfully updated */
    UbxNavSvinfoUp = 1 << 4, /* NavSvinfo message is successfully updated */
    UbxNavTimeGpsUp = 1 << 5
};

enum
{
    NAV5_dyn = 1 << 0,        /* Apply dynamic model settings */
    NAV5_minEl = 1 << 1,      /* Apply minimum elevation settings */
    NAV5_posFixMode = 1 << 2, /* Apply fix mode settings */
    NAV5_drLim = 1 << 3,      /* Reserved */
    NAV5_dposMask = 1 << 4,   /* Apply position mask settings */
    NAV5_timeMask = 1 << 5,   /* Apply time mask settings */
    NAV5_sHoldMask = 1 << 6,  /* Apply static hold settings */
    NAV5_dgpsMask = 1 << 7    /* Apply DGPS settings */
};

#pragma pack(push, 1)

typedef struct
{
    uint16_t mask;              /* Parameters bitmask */
    uint8_t dyn_model;          /* Dynamic model */
    uint8_t fix_mode;           /* Position Fixing Mode */
    int32_t fixed_alt;          /* Fixed altitude (mean sea level) for 2D fix mode. */
    uint32_t fixed_alt_var;     /* Fixed altitude variance for 2D mode. */
    int8_t min_elev;            /* Minimum Elevation for a GNSS satellite to be used in NAV */
    uint8_t dr_limit;           /* Reserved */
    uint16_t p_dop;             /* Position DOP Mask to use */
    uint16_t t_dop;             /* Time DOP Mask to use */
    uint16_t p_acc;             /* Position Accuracy Mask */
    uint16_t t_acc;             /* Time Accuracy Mask */
    uint8_t static_hold_thresh; /* Static hold threshold */
    uint8_t dgps_timeout;       /* DGPS timeout */
    uint8_t cno_thresh_numSVs;  /* Num sats required to have C/N0 above cnoThresh for a fix to be attempted */
    uint8_t cno_thresh;         /* C/N0 threshold for deciding whether to attempt a fix */
    uint16_t reserved2;         /* Reserved */
    uint32_t reserved3;         /* Reserved */
    uint32_t reserved4;         /* Reserved */
} CfgNav5;

typedef struct
{
    uint16_t meas_rate;
    uint16_t nav_rate;
    uint16_t time_ref;
} CfgRate;

typedef struct
{
    uint8_t portid;        /* Port ID mask */
    uint8_t resv1;         /* Reserved 1 */
    uint16_t txready;      /* Tx ready pin config */
    uint32_t mode;         /* Mode */
    uint32_t baudrate;     /* Baudrate register */
    uint16_t inprotomask;  /* In protocol mask */
    uint16_t outprotomask; /* Out protocol mask */
    uint16_t flags;        /* Flags bitfield */
    uint16_t resv5;        /* Reserved 5 */
} CfgPrt;

typedef struct
{
    uint8_t tp_idx;
    uint8_t reserved0;
    uint16_t reserved1;
    int16_t ant_cable_delay;
    int16_t rf_group_delay;
    uint32_t freq_period;
    uint32_t freq_period_lock;
    uint32_t pulse_len_ratio;
    uint32_t pulse_len_ratio_lock;
    int32_t user_config_delay;
    uint32_t flags;
} CfgTP5;

#pragma pack(pop)

void ubxInit(UbxState *ubx, uint32_t baudrate);
void ubxPoll(UbxState *ubx);

bool ubxGetStReadyStat(UbxState *ubx, uint16_t st_bf);
void ubxResetStReadyStat(UbxState *ubx, uint16_t st_bf);

#if __cplusplus
}
#endif
