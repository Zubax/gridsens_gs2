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
#include <crdr_chibios/config/config.hpp>
#include <unistd.h>

namespace gnss
{
namespace
{

crdr_chibios::config::Param<float> param_gnss_aux_rate("gnss_aux_rate_hz", 0.5, 0.1, 1.0);


void publishFix(const uavcan::UtcTime& ts_utc, const UbxState& state)
{
    const auto& data = state.fix;

    static uavcan::equipment::gnss::Fix msg;
    msg = uavcan::equipment::gnss::Fix();

    // Timestamp - Network clock, not GPS clock
    msg.timestamp = ts_utc;

    // Position
    msg.alt_1e2 = static_cast<uint32_t>(data.altitude  * 1e2F);
    msg.lat_1e7 = static_cast<uint32_t>(data.latitude  * 1e7F);
    msg.lon_1e7 = static_cast<uint32_t>(data.longitude * 1e7F);

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
    unsigned aux_rate_usec;

    mutable UbxState state;

    void tryInit() const
    {
        state = UbxState();

        ::sleep(1);
        sdStop(serial_port);
        sdStart(serial_port, &SerialConfig9600);   // Default serial port config
        ubxInit(&state, 115200);

        ::sleep(1);
        sdStop(serial_port);
        sdStart(serial_port, &SerialConfig115200); // New serial port config
        ubxInit(&state, 115200);                   // Reinit again in case if the port was configured at 115200
    }

    void handleTimeSync(const uavcan::MonotonicTime& ts_mono, const uavcan::UtcTime& ts_utc) const
    {
        if (state.time.valid &&
            state.time.utc_usec > 0 &&
            ts_mono.toMSec() > 5000)
        {
            node::adjustUtcTimeFromLocalSource(uavcan::UtcTime::fromUSec(state.time.utc_usec) - ts_utc);
        }
    }

    void tryRun() const
    {
        const unsigned ReportTimeoutMSec = 1100;
        auto prev_fix_report_at = uavcan_stm32::clock::getMonotonic();
        auto prev_aux_report_at = prev_fix_report_at;

        while (true)
        {
            // TODO: Proper ublox packet timestamping. Requires adequate parser.
            const auto ts_mono = uavcan_stm32::clock::getMonotonic();
            const auto ts_utc = uavcan_stm32::clock::getUtc();
            ubxPoll(&state);

            if (ubxGetStReadyStat(&state, TimeSt))
            {
                handleTimeSync(ts_mono, ts_utc);
                ubxResetStReadyStat(&state, TimeSt);
            }

            if (ubxGetStReadyStat(&state, FixSt))
            {
                publishFix(ts_utc, state);
                prev_fix_report_at = ts_mono;
                ubxResetStReadyStat(&state, FixSt);
            }
            else if (ubxGetStReadyStat(&state, DopSt))
            {
                // Fix and Aux will never be published within the same poll, that's intentional.
                ubxResetStReadyStat(&state, DopSt);
                if ((ts_mono - prev_aux_report_at).toUSec() >= (aux_rate_usec - 10000))
                {
                    prev_aux_report_at = ts_mono;
                    publishAux(state);
                }
            }
            else
            {
                ; // Nothing to do
            }

            if ((ts_mono - prev_fix_report_at).toMSec() > ReportTimeoutMSec)
            {
                node::setComponentStatus(node::ComponentID::Gnss, uavcan::protocol::NodeStatus::STATUS_CRITICAL);
                break;
            }
            else if (state.fix.sats_used < 6)
            {
                node::setComponentStatus(node::ComponentID::Gnss, uavcan::protocol::NodeStatus::STATUS_WARNING);
            }
            else
            {
                node::setComponentStatus(node::ComponentID::Gnss, uavcan::protocol::NodeStatus::STATUS_OK);
            }
        }
    }

public:
    msg_t main() override
    {
        aux_rate_usec = 1e6F / param_gnss_aux_rate.get();

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
