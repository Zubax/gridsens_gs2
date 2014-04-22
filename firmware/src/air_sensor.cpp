/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "air_sensor.hpp"
#include "board/ms5611.h"
#include "node.hpp"

#include <uavcan/equipment/airdata/StaticAirData.hpp>

#include <ch.hpp>
#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/config/config.hpp>
#include <unistd.h>

namespace air_sensor
{
namespace
{

crdr_chibios::config::Param<float> param_pressure_variance("pressure_variance_pa2", 100.0, 1.0, 4000.0);
crdr_chibios::config::Param<float> param_temperature_variance("temperature_variance_degc2", 10.0, 1.0, 100.0);

class AirSensorThread : public chibios_rt::BaseStaticThread<1024>
{
    const unsigned PeriodUSec = 100000;

    float pressure_variance = 0;
    float temperature_variance = 0;

    mutable ::Ms5611 sens = ::Ms5611();

    void publish(const uavcan::UtcTime& timestamp, float pressure_pa, float temperature_degc) const
    {
        if (!node::isStarted())
        {
            return;
        }

        static uavcan::equipment::airdata::StaticAirData air_data;
        air_data.timestamp = timestamp;

        air_data.static_pressure = pressure_pa;
        air_data.static_pressure_variance = pressure_variance;

        air_data.static_temperature = temperature_degc;
        air_data.static_temperature_variance = temperature_variance;

        node::Lock locker;
        auto& node = node::getNode();

        static uavcan::Publisher<uavcan::equipment::airdata::StaticAirData> air_data_pub(node);
        (void)air_data_pub.broadcast(air_data);
    }

    void tryRun() const
    {
        systime_t sleep_until = chibios_rt::System::getTime();
        while (true)
        {
            sleep_until += US2ST(PeriodUSec);

            const uavcan::UtcTime timestamp = uavcan_stm32::clock::getUtc();
            int32_t raw_pressure = 0;
            int32_t raw_temperature = 0;
            if (!ms5611ReadPT(&sens, &raw_pressure, &raw_temperature))
            {
                break;
            }

            const float pressure = static_cast<float>(raw_pressure);
            const float temperature = raw_temperature / 100.F;

            publish(timestamp, pressure, temperature);

            chibios_rt::BaseThread::sleepUntil(sleep_until);
        }
    }

public:
    msg_t main() override
    {
        pressure_variance = param_pressure_variance.get();
        temperature_variance = param_temperature_variance.get();

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
