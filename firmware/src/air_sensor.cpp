/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <ch.hpp>
#include <crdr_chibios/sys/sys.h>
#include <unistd.h>

#include <uavcan/equipment/airdata/StaticAirData.hpp>
#include <uavcan/equipment/airdata/AltitudeAndClimbRate.hpp>

#include "air_sensor.hpp"
#include "uavcan.hpp"
#include "drivers/ms5611.h"

namespace app
{
namespace
{

void publish(float pressure_pa, float temperature_degc)
{
    if (!isUavcanNodeStarted())
    {
        return;
    }

    uavcan::equipment::airdata::StaticAirData air_data;
    air_data.timestamp = uavcan_stm32::clock::getUtc();
    air_data.static_pressure = pressure_pa;
    air_data.static_pressure_variance = 10.0;
    air_data.static_temperature = temperature_degc;
    air_data.static_temperature_variance = 2.0;

    UavcanLock locker;
    UavcanNode& node = getUavcanNode();

    static uavcan::Publisher<uavcan::equipment::airdata::StaticAirData> air_data_pub(node);
    //static uavcan::Publisher<uavcan::equipment::airdata::AltitudeAndClimbRate> alt_climb_pub(node);  // TODO

    (void)air_data_pub.broadcast(air_data);
}

class AirSensorThread : public chibios_rt::BaseStaticThread<1024>
{
public:
    msg_t main() override
    {
        auto sens = ::MS5611_t();

        MS5611_Reset(&sens);
        ::usleep(3000);                // Waiting for reset

        (void)MS5611_Get_Prom(&sens);  // TODO: handling PROM correctness

        while (true)
        {
            ::usleep(100000);          // TODO: rate

            int32_t raw_pressure = 0;
            int32_t raw_temperature = 0;
            MS5611_Read_PT(&sens, &raw_pressure, &raw_temperature);

            publish(static_cast<float>(raw_pressure), raw_temperature / 100.f);
        }

        return msg_t();
    }
} air_sensor_thread;

}

int airSensorInit()
{
    (void)air_sensor_thread.start(HIGHPRIO - 10);
    return 0;
}

}
