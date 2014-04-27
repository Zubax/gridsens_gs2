/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "node.hpp"

#include "board/board.hpp"
#include <ch.hpp>
#include <unistd.h>

#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>

#include <crdr_chibios/config/config.hpp>
#include <crdr_chibios/sys/sys.h>

namespace node
{
namespace
{

const unsigned TimeSyncPubPeriodMSec = 1000;
const unsigned IfaceLedUpdatePeriodMSec = 25;

crdr_chibios::config::Param<unsigned> param_can_bitrate("can_bitrate", 1000000, 20000, 1000000);
crdr_chibios::config::Param<unsigned> param_node_id("uavcan_node_id", 125, 1, 125);
crdr_chibios::config::Param<bool> param_time_sync_master_on("time_sync_master_on", true);

uavcan_stm32::CanInitHelper<> can;

uavcan_stm32::Mutex node_mutex;

ComponentStatusManager comp_stat_mgr(uavcan::protocol::NodeStatus::STATUS_INITIALIZING);

bool started = false;
bool local_utc_updated = false;

void configureNode()
{
    Node& node = getNode();

    node.setNodeID(param_node_id.get());
    node.setName("com.courierdrone.gps");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    node.setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    hwver.major = HW_VERSION;
    node.setHardwareVersion(hwver);
}

void configureClockSync()
{
    // TODO: Hardware support for clock sync via PPS line
    // STM32 driver needs better clock speed adjustment algorithm
    auto params = uavcan_stm32::clock::getUtcSyncParams();

    params.p     = 0.00001;
    params.i_fwd = 0.000001;
    params.i_rev = 0.00001;

    params.rate_error_corner_freq = 0.000001;
    params.max_rate_correction_ppm = 70;

    params.min_jump = uavcan::UtcDuration::fromMSec(100);

    uavcan_stm32::clock::setUtcSyncParams(params);
}

uavcan::GlobalTimeSyncMaster& getTimeSyncMaster()
{
    static uavcan::GlobalTimeSyncMaster master(getNode());
    return master;
}

uavcan::GlobalTimeSyncSlave& getTimeSyncSlave()
{
    static uavcan::GlobalTimeSyncSlave ts(getNode());
    return ts;
}

bool isLocalUtcSourceEnabled()
{
    if (getTimeSyncSlave().isActive())
    {
        return getNode().getNodeID() < getTimeSyncSlave().getMasterNodeID();
    }
    return true;
}

void publishTimeSync(const uavcan::TimerEvent&)
{
    if (isLocalUtcSourceEnabled())
    {
        getTimeSyncSlave().suppress(local_utc_updated);
    }
    else
    {
        getTimeSyncSlave().suppress(false);
    }
    local_utc_updated = false;

    if (uavcan_stm32::clock::isUtcLocked())
    {
        (void)getTimeSyncMaster().publish();
    }
}

/*
 * UAVCAN spin loop
 */
class : public chibios_rt::BaseStaticThread<3000>
{
    void init() const
    {
        configureNode();
        configureClockSync();

        // Starting the UAVCAN node
        while (true)
        {
            {
                Lock locker;
                const int uavcan_start_res = getNode().start();
                if (uavcan_start_res >= 0)
                {
                    break;
                }
                lowsyslog("Node init failure: %i, will retry\n", uavcan_start_res);
            }
            ::sleep(3);
        }
        assert(getNode().isStarted());

        // Starting the time sync slave
        while (true)
        {
            const int res = getTimeSyncSlave().start();
            if (res >= 0)
            {
                break;
            }
            lowsyslog("Time sync slave init failure: %i, will retry\n", res);
            ::sleep(1);
        }

        // Time sync master - if enabled
        if (param_time_sync_master_on.get())
        {
            while (true)
            {
                const int res = getTimeSyncMaster().init();
                if (res >= 0)
                {
                    break;
                }
                lowsyslog("Time sync master init failure: %i, will retry\n", res);
                ::sleep(1);
            }

            static uavcan::Timer tsm_timer(getNode());
            tsm_timer.setCallback(&publishTimeSync);
            tsm_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(TimeSyncPubPeriodMSec));
        }

        started = true;
        lowsyslog("UAVCAN node started\n");
    }

public:
    msg_t main() override
    {
        init();

        static uavcan::MonotonicTime prev_led_update;
        auto& node = getNode();

        while (true)
        {
            {
                Lock locker;

                node.getNodeStatusProvider().setStatusCode(comp_stat_mgr.getWorstStatusCode());

                const int spin_res = node.spin(uavcan::MonotonicDuration::fromUSec(500));
                if (spin_res < 0)
                {
                    lowsyslog("UAVCAN spin failure: %i\n", spin_res);
                }
            }

            // Iface LED update
            const auto ts = uavcan_stm32::clock::getMonotonic();
            if ((ts - prev_led_update).toMSec() >= IfaceLedUpdatePeriodMSec)
            {
                prev_led_update = ts;
                for (unsigned i = 0; i < can.driver.getNumIfaces(); i++)
                {
                    board::setCANLed(i, can.driver.getIface(i)->hadActivity());
                }
            }

            ::usleep(1000);
        }
        return msg_t();
    }
} node_thread;

}

Lock::Lock() : uavcan_stm32::MutexLocker(node_mutex) { }

bool isStarted()
{
    return started;
}

Node& getNode()
{
    static Node node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

void adjustUtcTimeFromLocalSource(const uavcan::UtcDuration& adjustment)
{
    Lock locker;
    if (isLocalUtcSourceEnabled())
    {
        getTimeSyncSlave().suppress(true);
        uavcan_stm32::clock::adjustUtc(adjustment);
        local_utc_updated = true;
    }
}

void setComponentStatus(ComponentID comp, ComponentStatusManager::StatusCode status)
{
    Lock locker;
    comp_stat_mgr.setComponentStatus(comp, status);
}

int init()
{
    const int can_res = can.init(param_can_bitrate.get());
    if (can_res < 0)
    {
        return can_res;
    }

    (void)node_thread.start(LOWPRIO);
    return 0;
}

}
