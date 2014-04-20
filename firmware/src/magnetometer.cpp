/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "magnetometer.hpp"
#include "board/hmc5883.h"
#include "node.hpp"

#include <uavcan/equipment/ahrs/Magnetometer.hpp>

#include <ch.hpp>
#include <crdr_chibios/sys/sys.h>
#include <unistd.h>

namespace magnetometer
{
namespace
{

void publish(float field[3])
{
    if (!node::isStarted())
    {
        lowsyslog("Magnetometer publication skipped: Node is not started\n");
        return;
    }

    uavcan::equipment::ahrs::Magnetometer mag;
    std::copy(field, field + 3, mag.magnetic_field.begin());
    mag.magnetic_field_covariance.push_back(0.1);

    node::Lock locker;
    auto& node = node::getNode();

    static uavcan::Publisher<uavcan::equipment::ahrs::Magnetometer> mag_pub(node);

    (void)mag_pub.broadcast(mag);
}

class MagThread : public chibios_rt::BaseStaticThread<1024>
{
public:
    msg_t main() override
    {
        auto sens = ::HMC5883_t();

        lowsyslog("HMC5883: Initializing...\n");
        ASSERT_ALWAYS(HMC5883_simple_init() >= 0);
        lowsyslog("HMC5883: Init OK\n");

        while (true)
        {
            ::usleep(1000000);

            auto meas = ::HMC5883meas_t();
            (void)HMC5883_readData(&sens, &meas);

            publish(meas.H);
        }

        return msg_t();
    }
} mag_thread;

}

void init()
{
    (void)mag_thread.start(HIGHPRIO - 8);
}

}
