/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "air_sensor.hpp"
#include "board/ms5611.h"
#include "node.hpp"

#include <uavcan/equipment/airdata/StaticAirData.hpp>
#include <uavcan/equipment/airdata/AltitudeAndClimbRate.hpp>

#include <ch.hpp>
#include <crdr_chibios/sys/sys.h>
#include <unistd.h>

namespace air_sensor
{
namespace
{

const unsigned PeriodUSec = 100000;

auto sens = ::Ms5611();

void publish(float pressure_pa, float temperature_degc)
{
    if (!node::isStarted())
    {
        return;
    }

    static uavcan::equipment::airdata::StaticAirData air_data;
    air_data.timestamp = uavcan_stm32::clock::getUtc();
    air_data.static_pressure = pressure_pa;
    air_data.static_pressure_variance = 10.0;
    air_data.static_temperature = temperature_degc;
    air_data.static_temperature_variance = 2.0;

    node::Lock locker;
    auto& node = node::getNode();

    static uavcan::Publisher<uavcan::equipment::airdata::StaticAirData> air_data_pub(node);
    //static uavcan::Publisher<uavcan::equipment::airdata::AltitudeAndClimbRate> alt_climb_pub(node);  // TODO

    (void)air_data_pub.broadcast(air_data);
}

void tryRun()
{
    systime_t sleep_until = chibios_rt::System::getTime();
    while (true)
    {
        sleep_until += US2ST(PeriodUSec);

        int32_t raw_pressure = 0;
        int32_t raw_temperature = 0;
        if (!ms5611ReadPT(&sens, &raw_pressure, &raw_temperature))
        {
            break;
        }

        const float pressure = static_cast<float>(raw_pressure);
        const float temperature = raw_temperature / 100.F;

        publish(pressure, temperature);

        chibios_rt::BaseThread::sleepUntil(sleep_until);
    }
}

class AirSensorThread : public chibios_rt::BaseStaticThread<1024>
{
public:
    msg_t main() override
    {
        while (true)
        {
            ::sleep(1);

            if (!ms5611Reset(&sens))
            {
                lowsyslog("Air sensor reset failed, will retry...\n");
                continue;
            }

            if (!ms5611GetProm(&sens))
            {
                lowsyslog("Air sensor PROM read failed, will retry...\n");
                continue;
            }

            tryRun();
            lowsyslog("Air sensor is about to restart...\n");
        }
        return msg_t();
    }
} air_sensor_thread;

}

void init()
{
    (void)air_sensor_thread.start(HIGHPRIO - 10);
}

}
