/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <ch.hpp>
#include <crdr_chibios/sys/sys.h>
#include <unistd.h>

#include <uavcan/equipment/gnss/RtcmStream.hpp>
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Aux.hpp>

#include "gnss.hpp"
#include "board/ublox.h"
#include "uavcan.hpp"

namespace app
{
namespace
{

t_GNSS ubx_state;


void publishFix()
{
    const t_GNSSfix& data = ubx_state.t_FIX;
    uavcan::equipment::gnss::Fix msg;

    // Timestamp - Network clock, not GPS clock
    msg.timestamp = uavcan_stm32::clock::getUtc();

    // Position
    msg.alt_1e2 = data.height    * 1e2;
    msg.lat_1e7 = data.latitude  * 1e7;
    msg.lon_1e7 = data.longitude * 1e7;

    // Velocity
    for (int i = 0; i < 3; i++)
    {
        msg.ned_velocity[i] = data.NEDspeed[i];
    }

    // Uncertainty
    msg.sats_used = data.satqty;

    if (data.fix == GNSSFIX_TIME)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY;
    }
    else if (data.fix == GNSSFIX_2D)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_2D_FIX;
    }
    else if (data.fix == GNSSFIX_3D)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_3D_FIX;
    }
    else
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_NO_FIX;
    }

    if (msg.status > uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY)
    {
        msg.velocity_covariance.push_back(data.speedCM[0]);
        msg.velocity_covariance.push_back(data.speedCM[4]);
        msg.velocity_covariance.push_back(data.speedCM[8]);

        msg.position_covariance.push_back(data.posCM[0]);
        msg.position_covariance.push_back(data.posCM[4]);
        msg.position_covariance.push_back(data.posCM[8]);
    }

    // Publishing
    UavcanLock locker;
    static uavcan::Publisher<uavcan::equipment::gnss::Fix> pub(getUavcanNode());
    (void)pub.broadcast(msg);
}

void publishAux()
{
    uavcan::equipment::gnss::Aux msg;

    msg.gdop = ubx_state.t_DOP.gdop;
    msg.hdop = ubx_state.t_DOP.hdop;
    msg.pdop = ubx_state.t_DOP.pdop;
    msg.tdop = ubx_state.t_DOP.tdop;
    msg.vdop = ubx_state.t_DOP.vdop;

    msg.sats_used = ubx_state.t_FIX.satqty;
    msg.sats_visible = msg.sats_used;        // FIXME

    // Publishing
    UavcanLock locker;
    static uavcan::Publisher<uavcan::equipment::gnss::Aux> pub(getUavcanNode());
    (void)pub.broadcast(msg);
}


void poll()
{
    static uavcan::MonotonicTime prev_fix;
    static uavcan::MonotonicTime prev_aux;

    if (isUavcanNodeStarted())
    {
        if (ubx_state.t_FIX.satqty < 6)
        {
            getUavcanNode().setStatusWarning();
        }
        else
        {
            getUavcanNode().setStatusOk();
        }

        const uavcan::MonotonicTime time = uavcan_stm32::clock::getMonotonic();
        if ((time - prev_fix).toMSec() >= 500)
        {
            prev_fix = time;
            publishFix();
        }
        if ((time - prev_aux).toMSec() >= 5000)
        {
            prev_aux = time;
            publishAux();
        }
    }
}


class GnssThread : public chibios_rt::BaseStaticThread<3000>
{
public:
    msg_t main() override
    {
        static const SerialConfig serial_cfg_9600   = { 9600,   0, USART_CR2_STOP1_BITS, 0 };
//        static const SerialConfig serial_cfg_115200 = { 115200, 0, USART_CR2_STOP1_BITS, 0 };

        static uint8_t buffer[512];

        // Default serial port config
        sdStart(&SD2, &serial_cfg_9600);

        // Init UBX (CREATE AND SEND CFG-PRT MESSAGE)
        ubx_init(&ubx_state, 9600, UBX_PARITY_NO, UBX_STOP_ONE, 0x00);

        // Reinit the serial port
//        sdStop(&SD2);
//        sdStart(&SD2, &serial_cfg_115200);

        while (true)
        {
            const int16_t sz = sdReadTimeout(&SD2, buffer, sizeof(buffer), TIME_INFINITE);
            if (sz > 0)
            {
                int16_t ubh_idx = -1;
                do
                {
                    ubh_idx++;
                    ubh_idx = ubx_findheader(buffer, sz - ubh_idx, ubh_idx);
                    if (ubh_idx >= 0)
                    {
                        ubx_parse(&ubx_state, buffer + ubh_idx, sz - ubh_idx);
                        poll();
                    }
                }
                while (ubh_idx >= 0);
            }
        }
        return msg_t();
    }
} gnss_thread;

}

int gnssInit()
{
    (void)gnss_thread.start(HIGHPRIO - 5);
    return 1;
}

}
