/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "magnetometer.hpp"
#include "node.hpp"

#include <array>

#include <uavcan/equipment/ahrs/Magnetometer.hpp>

#include <ch.hpp>
#include <zubax_chibios/sys/sys.h>
#include <zubax_chibios/config/config.hpp>
#include <zubax_chibios/watchdog/watchdog.hpp>
#include <unistd.h>

namespace magnetometer
{
namespace
{

const float AbsMaxValidGauss = 1.3;                                             ///< For the default gain
const auto MaxZeroVectorDuration = uavcan::MonotonicDuration::fromMSec(5000);   ///< Should be OK

const float GaussScale = 0.92e-03;

zubax_chibios::config::Param<float> param_variance("mag_variance_ga2", 0.005, 1e-6, 1.0);
zubax_chibios::config::Param<unsigned> param_rate("mag_rate_hz", 20, 1, 50);

void publish(float field[3], float variance)
{
    if (!node::isStarted())
    {
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

template <unsigned TxSize, unsigned RxSize>
bool io(const std::array<uint8_t, TxSize>& tx, std::array<uint8_t, RxSize>& rx)
{
    const unsigned Address = 0x1E;
    i2cAcquireBus(&I2CD2);
    const msg_t status = i2cMasterTransmitTimeout(&I2CD2, Address, tx.data(), TxSize, rx.data(), RxSize, MS2ST(5));
    i2cReleaseBus(&I2CD2);
    return status == RDY_OK;
}

bool tryInit()
{
    const uint8_t cfg_registers[] =
    {
        0b01111000, // Reg A: Averaging 8x, Update rate 75Hz, Normal mode
        0b00100000, // Reg B: Default gain
        0b00000000  // Mode: Continuous measurement
    };
    // Config write
    {
        std::array<uint8_t, 4> tx;
        tx[0] = 0;
        std::copy(cfg_registers, cfg_registers + 3, tx.begin() + 1);
        std::array<uint8_t, 0> rx;
        if (!io(tx, rx))
        {
            return false;
        }
    }
    // Config readback - make sure it was written correctly
    {
        std::array<uint8_t, 1> tx;
        tx[0] = 0;
        std::array<uint8_t, 3> rx;
        if (!io(tx, rx))
        {
            return false;
        }
        if (!std::equal(rx.begin(), rx.end(), cfg_registers))
        {
            return false;
        }
    }
    // TODO: Sensor self-test
    return true;
}

bool tryRead(float out_gauss[3])
{
    std::array<uint8_t, 1> tx;
    tx[0] = 3;
    std::array<uint8_t, 6> rx;
    if (!io(tx, rx))
    {
        return false;
    }

    const int16_t hx = (((int16_t)rx[0]) << 8) | rx[1];  // X
    const int16_t hz = (((int16_t)rx[2]) << 8) | rx[3];  // Z
    const int16_t hy = (((int16_t)rx[4]) << 8) | rx[5];  // Y

    out_gauss[0] = hx * GaussScale;
    out_gauss[1] = hy * GaussScale;
    out_gauss[2] = hz * GaussScale;
    return true;
}

void transformToNEDFrame(float inout_mag_vector[3])
{
    const float x = -inout_mag_vector[1];
    const float y = inout_mag_vector[0];
    const float z = inout_mag_vector[2];

    inout_mag_vector[0] = x;
    inout_mag_vector[1] = y;
    inout_mag_vector[2] = z;
}

class MagThread : public chibios_rt::BaseStaticThread<1024>
{
    uavcan::MonotonicTime last_nonzero_vector_ts_;

    static void setStatus(unsigned status)
    {
        node::setComponentStatus(node::ComponentID::Magnetometer, status);
    }

    unsigned estimateStatusFromMeasurement(const float (&vector)[3])
    {
        // Checking if measured vector is a zero vector. Zero vectors are suspicious.
        bool zero_vector = true;
        for (float v : vector)
        {
            if (std::abs(v) > 1e-9)
            {
                zero_vector = false;
                break;
            }
        }

        // If the measured vector is zero-length, we need to make sure it wasn't this way for too long.
        if (zero_vector && !last_nonzero_vector_ts_.isZero())
        {
            auto zero_vector_duration = uavcan_stm32::clock::getMonotonic() - last_nonzero_vector_ts_;
            if (zero_vector_duration > MaxZeroVectorDuration)
            {
                return uavcan::protocol::NodeStatus::STATUS_WARNING;
            }
        }
        else
        {
            last_nonzero_vector_ts_ = uavcan_stm32::clock::getMonotonic();
        }

        // Check if the vector components are within valid range.
        for (float v : vector)
        {
            if (std::abs(v) > AbsMaxValidGauss)
            {
                return uavcan::protocol::NodeStatus::STATUS_WARNING;
            }
        }

        return uavcan::protocol::NodeStatus::STATUS_OK;
    }

public:
    msg_t main() override
    {
        zubax_chibios::watchdog::Timer wdt;
        wdt.startMSec(1000);
        setName("mag");

        while (!tryInit())
        {
            setStatus(uavcan::protocol::NodeStatus::STATUS_CRITICAL);
            lowsyslog("Mag init failed, will retry...\n");
            ::usleep(500000);
            wdt.reset();
        }

        const float variance = param_variance.get();
        const uint64_t period_usec = 1000000 / param_rate.get();

        systime_t sleep_until = chibios_rt::System::getTime();

        while (true)
        {
            sleep_until += US2ST(period_usec);

            float vector[3] = {0, 0, 0};
            if (tryRead(vector))
            {
                transformToNEDFrame(vector);
                publish(vector, variance);
                setStatus(estimateStatusFromMeasurement(vector));
            }
            else
            {
                setStatus(uavcan::protocol::NodeStatus::STATUS_CRITICAL);
            }

            sysSleepUntilChTime(sleep_until);
            wdt.reset();
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
