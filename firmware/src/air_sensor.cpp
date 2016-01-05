/*
 * Copyright (C) 2014-2015  Zubax Robotics  <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "air_sensor.hpp"
#include "board/ms5611.h"
#include "node.hpp"

#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>

#include <ch.hpp>
#include <zubax_chibios/os.hpp>
#include <unistd.h>

namespace air_sensor
{
namespace
{

const float DegreesCelsiusToKelvinOffset = 273.15F;

const float ValidPressureRangePa[] = { 1000, 120000 };            ///< Sensor range
const float ValidTemperatureRangeDegC[] = { -40, 85 };            ///< Sensor range
const float OperatingTemperatureRangeDegC[] = { -30, 60 };        ///< Operating temperature, by specification

const unsigned MinPublicationPeriodUSec = unsigned(1e6 / 30);

os::config::Param<unsigned> param_period_usec("uavcan.pubp-pres",
                                              0, 0, 1000000);

os::config::Param<unsigned> param_prio("uavcan.prio-pres",
                                       16,
                                       uavcan::TransferPriority::NumericallyMin,
                                       uavcan::TransferPriority::NumericallyMax);

os::config::Param<float> param_pressure_variance("pres.variance", 100.0, 1.0, 4000.0);
os::config::Param<float> param_temperature_variance("temp.variance", 4.0, 1.0, 100.0);

chibios_rt::Mutex last_sample_mutex;
Sample last_sample;

class AirSensorThread : public chibios_rt::BaseStaticThread<1024>
{
    float pressure_variance = 0;
    float temperature_variance = 0;

    mutable os::watchdog::Timer watchdog_;
    mutable ::Ms5611 sens = ::Ms5611();

    static bool isInRange(float value, const float range_min_max[2])
    {
        return (value >= range_min_max[0]) && (value <= range_min_max[1]);
    }

    void publish(float pressure_pa, float temperature_degc) const
    {
        {
            os::MutexLocker mlock(last_sample_mutex);
            last_sample.seq_id++;
            last_sample.pressure_pa = pressure_pa;
            last_sample.temperature_k = temperature_degc + DegreesCelsiusToKelvinOffset;
        }

        if (!node::isStarted())
        {
            return;
        }

        static uavcan::equipment::air_data::StaticPressure pressure;
        pressure.static_pressure = pressure_pa;
        pressure.static_pressure_variance = pressure_variance;

        static uavcan::equipment::air_data::StaticTemperature temperature;
        temperature.static_temperature = temperature_degc + DegreesCelsiusToKelvinOffset;
        temperature.static_temperature_variance = temperature_variance;

        node::Lock locker;
        auto& node = node::getNode();

        static uavcan::Publisher<uavcan::equipment::air_data::StaticPressure> pressure_pub(node);
        static uavcan::Publisher<uavcan::equipment::air_data::StaticTemperature> temperature_pub(node);

        EXECUTE_ONCE_NON_THREAD_SAFE
        {
            pressure_pub.setPriority(param_prio.get());
            temperature_pub.setPriority(param_prio.get());
        }

        (void)pressure_pub.broadcast(pressure);
        (void)temperature_pub.broadcast(temperature);
    }

    void tryRun() const
    {
        systime_t sleep_until = chibios_rt::System::getTime();

        assert(param_period_usec.get() > 0);
        const unsigned period_usec = std::max(MinPublicationPeriodUSec, param_period_usec.get());

        while (!node::hasPendingRestartRequest())
        {
            watchdog_.reset();
            sleep_until += US2ST(period_usec);

            int32_t raw_pressure = 0;
            int32_t raw_temperature = 0;
            if (!ms5611ReadPT(&sens, &raw_pressure, &raw_temperature))
            {
                break;
            }

            const float pressure_pa = static_cast<float>(raw_pressure);
            const float temperature_degc = raw_temperature / 100.F;

            publish(pressure_pa, temperature_degc);

            if (!isInRange(pressure_pa, ValidPressureRangePa) ||
                !isInRange(temperature_degc, ValidTemperatureRangeDegC))
            {
                node::setComponentHealth(node::ComponentID::AirSensor, uavcan::protocol::NodeStatus::HEALTH_ERROR);
            }
            else if (!isInRange(temperature_degc, OperatingTemperatureRangeDegC))
            {
                node::setComponentHealth(node::ComponentID::AirSensor, uavcan::protocol::NodeStatus::HEALTH_WARNING);
            }
            else
            {
                node::setComponentHealth(node::ComponentID::AirSensor, uavcan::protocol::NodeStatus::HEALTH_OK);
            }

            os::sleepUntilChTime(sleep_until);
        }
    }

public:
    void main() override
    {
        watchdog_.startMSec(1100);
        setName("air_sensor");

        pressure_variance = param_pressure_variance.get();
        temperature_variance = param_temperature_variance.get();

        while (!node::hasPendingRestartRequest())
        {
            watchdog_.reset();
            ::usleep(500000);
            watchdog_.reset();

            if (!ms5611Reset(&sens))
            {
                os::lowsyslog("Air sensor reset failed, will retry...\n");
                continue;
            }

            if (!ms5611GetProm(&sens))
            {
                os::lowsyslog("Air sensor PROM read failed, will retry...\n");
                continue;
            }

            os::lowsyslog("Air sensor init OK\n");

            node::markComponentInitialized(node::ComponentID::AirSensor);
            tryRun();
            if (!node::hasPendingRestartRequest())
            {
                node::setComponentHealth(node::ComponentID::AirSensor, uavcan::protocol::NodeStatus::HEALTH_ERROR);
                os::lowsyslog("Air sensor is about to restart...\n");
            }
        }

        os::lowsyslog("Air sensor driver terminated\n");
    }
} air_sensor_thread;

}

void init()
{
    if (param_period_usec.get() > 0)
    {
        (void)air_sensor_thread.start(HIGHPRIO - 10);
    }
    else
    {
        node::markComponentInitialized(node::ComponentID::AirSensor);
        os::lowsyslog("Air sensor disabled\n");
    }
}

Sample getLastSample()
{
    os::MutexLocker mlock(last_sample_mutex);
    return last_sample;
}

}
