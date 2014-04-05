/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <unistd.h>
#include <ch.hpp>

#include <crdr_chibios/config/config.hpp>
#include <crdr_chibios/sys/sys.h>

#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>

namespace app
{
namespace
{

crdr_chibios::config::Param<unsigned> can_bitrate("can_bitrate", 1000000, 20000, 1000000);
crdr_chibios::config::Param<unsigned> node_id("uavcan_node_id", 42, 1, 120);

uavcan_stm32::CanInitHelper<> can;

typedef uavcan::Node<UAVCAN_MEM_POOL_BLOCK_SIZE * 64> Node;

Node& getNode()
{
    static Node node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

uavcan::GlobalTimeSyncSlave& getTimeSyncSlave()
{
    static uavcan::GlobalTimeSyncSlave time_sync_slave(getNode());
    return time_sync_slave;
}

void configureNode()
{
    Node& node = app::getNode();

    node.setNodeID(node_id.get());
    node.setName("com.courierdrone.gps");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    node.setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    hwver.major = HW_VERSION;
    node.setHardwareVersion(hwver);
}

/*
 * UAVCAN spin loop
 */
class : public chibios_rt::BaseStaticThread<2048>
{
public:
    msg_t main()
    {
        configureNode();

        Node& node = app::getNode();

        /*
         * Initializing the UAVCAN node - this may take a while
         */
        while (true)
        {
            uavcan::NodeInitializationResult init_result;
            const int uavcan_start_res = node.start(init_result);

            if (uavcan_start_res < 0)
            {
                lowsyslog("Node initialization failure: %i, will try agin soon\n", uavcan_start_res);
            }
            else if (!init_result.isOk())
            {
                lowsyslog("Network conflict with %u, will try again soon\n", init_result.conflicting_node.get());
            }
            else
            {
                break;
            }
            ::sleep(3);
        }

        /*
         * Time synchronizer
         */
        ASSERT_ALWAYS(getTimeSyncSlave().start() >= 0);

        /*
         * Main loop
         */
        lowsyslog("UAVCAN node started\n");
        node.setStatusOk();
        while (true)
        {
            const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(5000));
            if (spin_res < 0)
            {
                lowsyslog("Spin failure: %i\n", spin_res);
            }
        }
        return msg_t();
    }
} node_thread;

}

int uavcanInit()
{
    const int can_res = can.init(can_bitrate.get());
    if (can_res < 0)
    {
        return can_res;
    }

    (void)node_thread.start(LOWPRIO);
    return 0;
}

}
