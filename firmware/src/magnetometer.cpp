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
#include <crdr_chibios/config/config.hpp>
#include <unistd.h>

namespace magnetometer
{
namespace
{

crdr_chibios::config::Param<float> param_variance("mag_variance", 0.1, 1e-6, 1.0);

void publish(float field[3], float variance)
{
    if (!node::isStarted())
    {
        lowsyslog("Magnetometer publication skipped: Node is not started\n");
        return;
    }

    uavcan::equipment::ahrs::Magnetometer mag;
    std::copy(field, field + 3, mag.magnetic_field.begin());
    mag.magnetic_field_covariance.push_back(variance);

    node::Lock locker;
    auto& node = node::getNode();

    static uavcan::Publisher<uavcan::equipment::ahrs::Magnetometer> mag_pub(node);

    (void)mag_pub.broadcast(mag);
}

class MagThread : public chibios_rt::BaseStaticThread<1024>
{
    HMC5883_t sensor = ::HMC5883_t();

public:
    msg_t main() override
    {
        lowsyslog("HMC5883: Initializing...\n");
        while (hmc5883Init(&sensor, Avg4samp, SPS_15, MeasModeNormal, Gain1090, OmCont) < 0)
        {
            ::sleep(1);
        }
        lowsyslog("HMC5883: Init OK\n");

        const float variance = param_variance.get();

        while (true)
        {
            ::usleep(100000);

            auto meas = ::HMC5883meas_t();
            (void)hmc5883ReadData(&sensor, &meas);

            publish(meas.h, variance);
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
