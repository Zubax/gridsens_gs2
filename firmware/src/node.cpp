/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <unistd.h>
#include <ch.hpp>

#include <crdr_chibios/config/config.hpp>
#include <crdr_chibios/sys/sys.h>

#include "node.hpp"

namespace node
{
namespace
{

crdr_chibios::config::Param<unsigned> can_bitrate("can_bitrate", 1000000, 20000, 1000000);
crdr_chibios::config::Param<unsigned> node_id("uavcan_node_id", 42, 1, 120);

uavcan_stm32::CanInitHelper<> can;

uavcan_stm32::Mutex node_mutex;

void configureNode()
{
    Node& node = getNode();

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
class : public chibios_rt::BaseStaticThread<3000>
{
public:
    msg_t main() override
    {
        configureNode();

        /*
         * Starting the UAVCAN node - this may take a few seconds
         */
        while (true)
        {
            {
                Lock locker;
                const int uavcan_start_res = getNode().start();
                if (uavcan_start_res >= 0)
                {
                    break;
                }
                lowsyslog("Node initialization failure: %i, will try agin soon\n", uavcan_start_res);
            }
            ::sleep(3);
        }
        getNode().setStatusOk();

        /*
         * Main loop
         */
        lowsyslog("UAVCAN node started\n");
        while (true)
        {
            {
                Lock locker;
                const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromUSec(500));
                if (spin_res < 0)
                {
                    lowsyslog("UAVCAN spin failure: %i\n", spin_res);
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
    return getNode().isStarted();
}

Node& getNode()
{
    static Node node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

int init()
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
