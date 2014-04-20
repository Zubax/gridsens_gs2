/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <ch.hpp>
#include <crdr_chibios/sys/sys.h>
#include <unistd.h>

#include <uavcan/equipment/ahrs/Magnetometer.hpp>

#include "magnetometer.hpp"
#include "uavcan.hpp"
#include "board/hmc5883.h"

namespace app
{
namespace
{

void publish(float field[3])
{
    if (!isUavcanNodeStarted())
    {
        lowsyslog("Magnetometer publication skipped: Node is not started\n");
        return;
    }

    uavcan::equipment::ahrs::Magnetometer mag;
    std::copy(field, field + 3, mag.magnetic_field.begin());
    mag.magnetic_field_covariance.push_back(0.1);

    UavcanLock locker;
    UavcanNode& node = getUavcanNode();

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

int magnetometerInit()
{
    (void)mag_thread.start(HIGHPRIO - 8);
    return 0;
}

}
