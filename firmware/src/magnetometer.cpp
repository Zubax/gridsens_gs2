/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "magnetometer.hpp"
#include "node.hpp"
#include "execute_once.hpp"

#include <array>

#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>

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

zubax_chibios::config::Param<float> param_variance("mag.variance", 0.005, 1e-6, 1.0);

zubax_chibios::config::Param<unsigned> param_period_usec("uavcan.pubp-mag",
                                                         20000, 20000, 1000000);

zubax_chibios::config::Param<unsigned> param_prio("uavcan.prio-mag",
                                                  16,
                                                  uavcan::TransferPriority::NumericallyMin,
                                                  uavcan::TransferPriority::NumericallyMax);

void publish(float field[3], float variance)
{
    if (!node::isStarted())
    {
        return;
    }

    uavcan::equipment::ahrs::MagneticFieldStrength mag;
    std::copy(field, field + 3, mag.magnetic_field_ga.begin());
    mag.magnetic_field_covariance.push_back(variance);

    node::Lock locker;
    auto& node = node::getNode();

    static uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength> mag_pub(node);

    EXECUTE_ONCE_NON_THREAD_SAFE
    {
        mag_pub.setPriority(param_prio.get());
    }

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

bool writeCraCrbMode(uint8_t cra, uint8_t crb, uint8_t mode)
{
    const uint8_t cfg_registers[] = { cra, crb, mode };
    // Write
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
    // Readback - make sure it was written correctly
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
    return true;
}

bool tryReadRawData(int16_t out_xyz[3])
{
    std::array<uint8_t, 1> tx;
    tx[0] = 3;
    std::array<uint8_t, 6> rx;
    if (!io(tx, rx))
    {
        ::lowsyslog("Mag read failed\n");
        return false;
    }
    out_xyz[0] = (((int16_t)rx[0]) << 8) | rx[1];  // X
    out_xyz[2] = (((int16_t)rx[2]) << 8) | rx[3];  // Z
    out_xyz[1] = (((int16_t)rx[4]) << 8) | rx[5];  // Y
    return true;
}

bool trySelfTest(const bool polarity)
{
    static const int16_t LowLimit  = 143;     // For gain 7
    static const int16_t HighLimit = 339;

    /*
     * Enable self test mode (pos/neg)
     */
    if (!writeCraCrbMode(polarity ? 0x71 : 0x72,    // Reg A: 8-average, 15 Hz default, positive/negative self test
                         0xE0,                      // Reg B: Gain=7
                         0x00))                     // Mode: Continuous-measurement mode
    {
        ::lowsyslog("Mag: Failed to begin self test\n");
        return false;
    }

    /*
     * Ignore the first two samples, keep the last one
     */
    int16_t raw_xyz[3] = {};
    for (int i = 0; i < 3; i++)
    {
        ::usleep(80000);
        if (!tryReadRawData(raw_xyz))
        {
            return false;
        }
    }

    ::lowsyslog("Mag self test sample, %s, x/y/z: %d %d %d\n", polarity ? "positive": "negative",
                int(raw_xyz[0]), int(raw_xyz[1]), int(raw_xyz[2]));

    /*
     * Validate the obtained results
     */
    for (const auto a : raw_xyz)
    {
        const auto normalized = polarity ? a : -a;
        if ((normalized < LowLimit) || (normalized > HighLimit))
        {
            ::lowsyslog("Mag self test sample %d is invalid\n", int(a));
            return false;
        }
    }

    return true;
}

bool tryInit()
{
    /*
     * Run two self tests - positive and negative
     */
    if (!trySelfTest(true))
    {
        ::lowsyslog("Mag positive self test failed\n");
        return false;
    }

    if (!trySelfTest(false))
    {
        ::lowsyslog("Mag negative self test failed\n");
        return false;
    }

    /*
     * Configure normal mode
     */
    if (!writeCraCrbMode(0b01111000,  // Reg A: Averaging 8x, Update rate 75Hz, Normal mode
                         0b00100000,  // Reg B: Default gain
                         0b00000000)) // Mode: Continuous measurement
    {
        ::lowsyslog("Mag: Failed to begin normal operation\n");
        return false;
    }

    /*
     * Discard the first sample after gain change
     */
    ::usleep(80000);
    int16_t dummy[3] = {};
    return tryReadRawData(dummy);
}

bool tryRead(float out_gauss[3])
{
    int16_t raw_xyz[3] = {};
    if (!tryReadRawData(raw_xyz))
    {
        return false;
    }
    out_gauss[0] = raw_xyz[0] * GaussScale;
    out_gauss[1] = raw_xyz[1] * GaussScale;
    out_gauss[2] = raw_xyz[2] * GaussScale;
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

    static void setStatus(std::uint8_t status)
    {
        node::setComponentHealth(node::ComponentID::Magnetometer, status);
    }

    std::uint8_t estimateStatusFromMeasurement(const float (&vector)[3])
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
                return uavcan::protocol::NodeStatus::HEALTH_WARNING;
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
                return uavcan::protocol::NodeStatus::HEALTH_WARNING;
            }
        }

        return uavcan::protocol::NodeStatus::HEALTH_OK;
    }

public:
    msg_t main() override
    {
        zubax_chibios::watchdog::Timer wdt;
        wdt.startMSec(1000);
        setName("mag");

        ::usleep(500000);         // Startup delay
        wdt.reset();

        node::markComponentInitialized(node::ComponentID::Magnetometer);

        while (!tryInit() && !node::hasPendingRestartRequest())
        {
            setStatus(uavcan::protocol::NodeStatus::HEALTH_ERROR);
            lowsyslog("Mag init failed, will retry...\n");
            ::usleep(500000);
            wdt.reset();
        }

        wdt.reset();

        const float variance = param_variance.get();
        const uint64_t period_usec = param_period_usec.get();

        systime_t sleep_until = chibios_rt::System::getTime();

        while (!node::hasPendingRestartRequest())
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
                setStatus(uavcan::protocol::NodeStatus::HEALTH_ERROR);
            }

            sysSleepUntilChTime(sleep_until);
            wdt.reset();
        }

        lowsyslog("Mag driver terminated\n");
        return msg_t();
    }
} mag_thread;

}

void init()
{
    (void)mag_thread.start(HIGHPRIO - 8);
}

}
