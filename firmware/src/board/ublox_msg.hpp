/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <cstdint>

namespace ublox
{

#pragma pack(1)

namespace msg
{

typedef std::uint8_t  U1;
typedef std::uint16_t U2;
typedef std::uint32_t U4;

typedef std::int8_t  I1;
typedef std::int16_t I2;
typedef std::int32_t I4;

typedef std::uint8_t  X1;
typedef std::uint16_t X2;
typedef std::uint32_t X4;

typedef float  R4;
typedef double R8;

typedef char CH;

enum class GnssID : U1
{
    GPS     = 0,
    SBAS    = 1,
    Galileo = 2,
    BeiDou  = 3,
    QZSS    = 5,
    GLONASS = 6
};

struct ACK_ACK
{
    static constexpr unsigned Class = 0x05;
    static constexpr unsigned ID    = 0x01;

    U1 clsID;
    U1 msgID;
};

struct ACK_NAK
{
    static constexpr unsigned Class = 0x05;
    static constexpr unsigned ID    = 0x00;

    U1 clsID;
    U1 msgID;
};

struct CFG_GNSS
{
    static constexpr unsigned Class = 0x06;
    static constexpr unsigned ID    = 0x3E;

    static constexpr unsigned MsgVersion = 0;

    U1 msgVer;
    U1 numTrkChHw;
    U1 numTrkChUse;
    U1 numConfigBlocks;

    struct ConfigBlock
    {
        GnssID gnssId;
        U1 resTrkCh;
        U1 maxTrkCh;
        U1 reserved1;
        X4 flags;
    } configBlocks[1];
};
static_assert(sizeof(CFG_GNSS) == 12, "Struct size error");

struct CFG_NAV5
{
    static constexpr unsigned Class = 0x06;
    static constexpr unsigned ID    = 0x24;

    X2 mask;

    struct Mask
    {
        static constexpr X2 dyn            = 1;
        static constexpr X2 minEl          = 2;
        static constexpr X2 posFixMode     = 4;
        static constexpr X2 drLim          = 8;
        static constexpr X2 posMask        = 16;
        static constexpr X2 timeMask       = 32;
        static constexpr X2 staticHoldMask = 64;
        static constexpr X2 dgpsMask       = 128;
        static constexpr X2 cnoThreshold   = 256;
        // Empty 512
        static constexpr X2 utc            = 1024;
    };

    enum class DynModel : U1
    {
        Portable    = 0,
        Stationary  = 2,
        Pedestrian  = 3,
        Automotive  = 4,
        Sea         = 5,
        Airborne_1g = 6,
        Airborne_2g = 7,
        Airborne_4g = 8
    } dynModel;

    enum class FixMode : U1
    {
        Fix2D    = 1,
        Fix3D    = 2,
        Auto2D3D = 3
    } fixMode;

    I4 fixedAlt;
    U4 fixedAltVar;
    I1 minElev;
    U1 drLimit;
    U2 pDop;
    U2 tDop;
    U2 pAcc;
    U2 tAcc;
    U1 staticHoldThresh;
    U1 dgpsTimeOut;
    U1 cnoThreshNumSVs;
    U1 cnoThresh;
    U2 reserved;
    U2 staticHoldMaxDist;

    enum class UtcStandard : U1
    {
        NotSpecified = 0,
        GPS          = 3,
        GLONASS      = 6,
        BeiDou       = 7
    } utcStandard;

    U1 reserved3;
    U4 reserved4;
};
static_assert(sizeof(CFG_NAV5) == 36, "Struct size error");


struct CFG_RATE
{
    static constexpr unsigned Class = 0x06;
    static constexpr unsigned ID    = 0x08;

    U2 measRate;
    U2 navRate;

    enum class TimeRef : U2
    {
        UTC = 0,
        GPS = 1
    } timeRef;
};
static_assert(sizeof(CFG_RATE) == 6, "Struct size error");


struct CFG_MSG
{
    static constexpr unsigned Class = 0x06;
    static constexpr unsigned ID    = 0x01;

    U1 msgClass;
    U1 msgID;
    U1 rate;
};
static_assert(sizeof(CFG_MSG) == 3, "Struct size error");


struct CFG_PRT_UART
{
    static constexpr unsigned Class = 0x06;
    static constexpr unsigned ID    = 0x00;

    enum class PortID : U1
    {
        DDC   = 0,
        UART1 = 1,
        USB   = 3,
        SPI   = 4
    } portID;

    U1 reserved0;
    X2 txReady;

    X4 mode;
    static constexpr X4 Mode8N1 = 0b00100011010000;  // Default mode

    U4 baudRate;
    X2 inProtoMask;
    X2 outProtoMask;
    X2 flags;
    U2 reserved5;

    struct InProtoMask
    {
        static constexpr X2 inUbx  = 1;
        static constexpr X2 inNmea = 2;
        static constexpr X2 inRtcm = 4;
    };

    struct OutProtoMask
    {
        static constexpr X2 outUbx  = 1;
        static constexpr X2 outNmea = 2;
    };
};
static_assert(sizeof(CFG_PRT_UART) == 20, "Struct size error");


struct NAV_DOP
{
    static constexpr unsigned Class = 0x01;
    static constexpr unsigned ID    = 0x04;

    U4 iTOW;
    U2 gDOP;
    U2 pDOP;
    U2 tDOP;
    U2 vDOP;
    U2 hDOP;
    U2 nDOP;
    U2 eDOP;
};
static_assert(sizeof(NAV_DOP) == 18, "Struct size error");


struct NAV_PVT
{
    static constexpr unsigned Class = 0x01;
    static constexpr unsigned ID    = 0x07;

    U4 iTOW;
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 min;
    U1 sec;
    X1 valid;

    struct ValidMask
    {
        static constexpr X1 validDate     = 1;
        static constexpr X1 validTime     = 2;
        static constexpr X1 fullyResolved = 4;
    };

    U4 tAcc;
    I4 nano;

    enum class FixType : U1
    {
        NoFix                 = 0,
        DeadReckoning         = 1,
        Fix2D                 = 2,
        Fix3D                 = 3,
        GnssAndDeadReckoning = 4,
        TimeOnly              = 5
    } fixType;

    X1 flags;

    struct FlagsMask
    {
        static constexpr X1 gnssFixOK    = 1;
        static constexpr X1 diffSoln     = 2;
        static constexpr X1 headVehValid = 32;
    };

    U1 reserved1;
    U1 numSV;
    I4 lon;
    I4 lat;
    I4 height;
    I4 hMSL;
    U4 hAcc;
    U4 vAcc;
    I4 velN;
    I4 velE;
    I4 velD;
    I4 gSpeed;
    I4 headMot;
    U4 sAcc;
    U4 headAcc;
    U2 pDOP;
    X2 reserved2;
    X4 reserved3;
    I4 headVeh;
    X4 reserved4;
};
static_assert(sizeof(NAV_PVT) == 92, "Struct size error");


struct NAV_SAT
{
    static constexpr unsigned Class = 0x01;
    static constexpr unsigned ID    = 0x35;

    static constexpr unsigned MsgVersion = 1;

    U4 iTOW;
    U1 version;
    U1 numSvs;
    U2 reserved2;

    struct Sv
    {
        GnssID gnssId;
        U1 svId;
        U1 cno;
        I1 elev;
        I2 azim;
        I2 prRes;
        X4 flags;

        struct FlagsMask
        {
            static constexpr X4 svUsed   = 1 << 3;
            static constexpr X4 diffCorr = 1 << 6;
            static constexpr X4 smoothed = 1 << 7;
            static constexpr X4 ephAvail = 1 << 11;
            static constexpr X4 almAvail = 1 << 12;
            static constexpr X4 anoAvail = 1 << 13;
            static constexpr X4 aopAvail = 1 << 14;
        };

        enum class SignalQualityIndicator
        {
            NoSignal                  = 0,
            SearchingSignal           = 1,
            SignalAcquired            = 2,
            SignalDetectedButUnusable = 3,
            CodeLockOnSignal          = 4,
            CodeAndCarrierLocked      = 5,
            CodeAndCarrierLocked_1    = 6,
            CodeAndCarrierLocked_2    = 7
        };

        enum class SvHealth
        {
            Unknown = 0,
            Healthy = 1,
            Unhealthy = 2
        };
    } svs[1];
};
static_assert(sizeof(NAV_SAT) == 20, "Struct size error");


struct MON_GNSS
{
    static constexpr unsigned Class = 0x0A;
    static constexpr unsigned ID    = 0x28;

    static constexpr unsigned MsgVersion = 1;

    struct GnssMask
    {
        static constexpr unsigned GPS     = 1;
        static constexpr unsigned GLONASS = 2;
        static constexpr unsigned BeiDou  = 4;
    };

    U1 version;
    X1 supported;
    X1 default_;
    X1 enabled;
    U1 simultaneous;
    U1 reserved4[3];
};
static_assert(sizeof(MON_GNSS) == 8, "Struct size error");

} // namespace msg

#pragma pack(0)

}
