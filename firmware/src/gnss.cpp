/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "gnss.hpp"
#include "board/ublox.h"
#include "node.hpp"

#include <uavcan/equipment/gnss/RtcmStream.hpp>
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Aux.hpp>

#include <ch.hpp>
#include <crdr_chibios/sys/sys.h>
#include <unistd.h>

namespace gnss
{
namespace
{

void publishFix(const UbxState& state)
{
    const auto& data = state.fix;

    static uavcan::equipment::gnss::Fix msg;
    msg = uavcan::equipment::gnss::Fix();

    // Timestamp - Network clock, not GPS clock
    msg.timestamp = uavcan_stm32::clock::getUtc();

    // Position
    msg.alt_1e2 = static_cast<uint32_t>(data.altitude  * 1e2);
    msg.lat_1e7 = static_cast<uint32_t>(data.latitude  * 1e7);
    msg.lon_1e7 = static_cast<uint32_t>(data.longitude * 1e7);

    // Velocity
    for (int i = 0; i < 3; i++)
    {
        msg.ned_velocity[i] = data.ned_speed[i];
    }

    // Uncertainty
    msg.sats_used = data.sats_used;

    if (data.fix == GNSSfix_Time)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY;
    }
    else if (data.fix == GNSSfix_2D)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_2D_FIX;
    }
    else if (data.fix == GNSSfix_3D)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_3D_FIX;
    }
    else
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_NO_FIX;
    }

    if (msg.status > uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY)
    {
        msg.velocity_covariance.packSquareMatrix(data.speed_cov);
        msg.position_covariance.packSquareMatrix(data.pos_cov);
    }

    // Publishing
    node::Lock locker;
    static uavcan::Publisher<uavcan::equipment::gnss::Fix> pub(node::getNode());
    (void)pub.broadcast(msg);
}

void publishAux(const UbxState& state)
{
    static uavcan::equipment::gnss::Aux msg;

    msg.gdop = state.dop.gdop;
    msg.hdop = state.dop.hdop;
    msg.pdop = state.dop.pdop;
    msg.tdop = state.dop.tdop;
    msg.vdop = state.dop.vdop;

    msg.sats_used = 0;
    msg.sats_visible = 0;
    for (const auto sat : state.sats.sat)
    {
        msg.sats_visible += (sat.sat_stmask == 0) ? 0 : 1;
        msg.sats_used += (sat.sat_stmask & UbxSat_Used) ? 1 : 0;
    }

    // Publishing
    node::Lock locker;
    static uavcan::Publisher<uavcan::equipment::gnss::Aux> pub(node::getNode());
    (void)pub.broadcast(msg);
}


const SerialConfig SerialConfig9600   = { 9600,   0, USART_CR2_STOP1_BITS, 0 };
const SerialConfig SerialConfig115200 = { 115200, 0, USART_CR2_STOP1_BITS, 0 };

auto* const serial_port = &SD2;

class GnssThread : public chibios_rt::BaseStaticThread<3000>
{
    UbxState state;

    void tryInit()
    {
        state = UbxState();

        ::sleep(1);
        sdStop(serial_port);
        sdStart(serial_port, &SerialConfig9600);   // Default serial port config
        ubxInit(&state, 115200);

//        ::sleep(1);
//        sdStop(serial_port);
//        sdStart(serial_port, &SerialConfig115200); // New serial port config
//        ubxInit(&state, 115200);                   // Reinit again in case if the port was configured at 115200
    }

    void tryRun()
    {
        const unsigned ReportTimeoutMSec = 1100;
        auto prev_fix_report_at = uavcan_stm32::clock::getMonotonic();
        while (true)
        {
            const auto ts = uavcan_stm32::clock::getMonotonic();

            ubxPoll(&state);

            if (ubxGetStReadyStat(&state, FixSt))
            {
                prev_fix_report_at = ts;
                ubxResetStReadyStat(&state, FixSt);
                publishFix(state);
            }
            else  // Fix and Aux will never be published within the same poll, that's intentional.
            {
                // TODO: Publish AUX at fixed rate 0.5 Hz
            }

            // TODO: update node status

            if ((ts - prev_fix_report_at).toMSec() > ReportTimeoutMSec)
            {
                // TODO: Set node status ERROR
                break;
            }
        }
    }

public:
    msg_t main() override
    {
        while (true)
        {
            ::sleep(1);
            lowsyslog("GNSS init...\n");
            tryInit();
            tryRun();
        }
        return msg_t();
    }
} gnss_thread;

}

void init()
{
    (void)gnss_thread.start(HIGHPRIO);
}

}
