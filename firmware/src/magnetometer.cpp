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

#include "magnetometer.hpp"
#include "node.hpp"

#include <array>

#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>

#include <ch.hpp>
#include <zubax_chibios/os.hpp>
#include <unistd.h>

namespace magnetometer
{
namespace
{

static auto& I2CD = I2CD1;

const float AbsMaxValidGauss = 8.1F;                                            ///< For the selected gain
const auto MaxZeroVectorDuration = uavcan::MonotonicDuration::fromMSec(5000);   ///< Should be OK

const float GaussScale = 4.35e-03;

os::config::Param<float> param_scaling_coef("mag.scaling_coef", 1.0F, 0.1F, 2.0F);

os::config::Param<float> param_variance("mag.variance", 0.005, 1e-6, 1.0);

os::config::Param<unsigned> param_period_usec("uavcan.pubp-mag", 20000, 10000, 1000000);

os::config::Param<unsigned> param_prio("uavcan.prio-mag",
                                       16,
                                       uavcan::TransferPriority::NumericallyMin,
                                       uavcan::TransferPriority::NumericallyMax);

class MedianFilter
{
    float a_ = 0.0F;
    float b_ = 0.0F;

public:
    float update(float x)
    {
        const float median = std::max(std::min(a_, b_),
                                      std::min(std::max(a_, b_), x));
        a_ = b_;
        b_ = x;
        return median;
    }
};

MedianFilter median_filters_xyz[3];
chibios_rt::Mutex last_sample_mutex;
Sample last_sample;

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
    i2cAcquireBus(&I2CD);
    const msg_t status = i2cMasterTransmitTimeout(&I2CD, Address, tx.data(), TxSize, rx.data(), RxSize, MS2ST(5));
#if defined(DEBUG_BUILD) && DEBUG_BUILD
    if (status != MSG_OK)
    {
        os::lowsyslog("Mag i2c st %d err %u\n", int(status), unsigned(i2cGetErrors(&I2CD)));
    }
#endif
    i2cReleaseBus(&I2CD);
    return status == MSG_OK;
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
        ::os::lowsyslog("Mag read failed\n");
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
    if (!writeCraCrbMode(polarity ? 0b11110001 : 0b11110010, // Reg A: Temp comp., 8-avg, 15 Hz default, pos/neg test
                         0b11100000,                         // Reg B: Gain=7
                         0b00000000))                        // Mode: Continuous-measurement mode
    {
        ::os::lowsyslog("Mag: Failed to begin self test\n");
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

    ::os::lowsyslog("Mag self test sample, %s, x/y/z: %d %d %d\n", polarity ? "positive": "negative",
                int(raw_xyz[0]), int(raw_xyz[1]), int(raw_xyz[2]));

    /*
     * Validate the obtained results
     */
    for (const auto a : raw_xyz)
    {
        const auto normalized = polarity ? a : -a;
        if ((normalized < LowLimit) || (normalized > HighLimit))
        {
            ::os::lowsyslog("Mag self test sample %d is invalid\n", int(a));
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
        ::os::lowsyslog("Mag positive self test failed\n");
        return false;
    }

    if (!trySelfTest(false))
    {
        ::os::lowsyslog("Mag negative self test failed\n");
        return false;
    }

    /*
     * Configure normal mode
     */
    if (!writeCraCrbMode(0b11111000,  // Reg A: Temp compens., Averaging 8x, Update rate 75Hz, Normal mode
                         0b11100000,  // Reg B: Minimum gain
                         0b00000000)) // Mode: Continuous measurement
    {
        ::os::lowsyslog("Mag: Failed to begin normal operation\n");
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

void rescale(float (&inout_mag_vector)[3], float coef)
{
    for (auto& x : inout_mag_vector)
    {
        x *= coef;
    }
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

void applyMedianFilter(float (&inout_mag_vector)[3])
{
    for (int i = 0; i < 3; i++)
    {
        inout_mag_vector[i] = median_filters_xyz[i].update(inout_mag_vector[i]);
    }
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
            auto zero_vector_duration = uavcan_stm32::SystemClock::instance().getMonotonic() - last_nonzero_vector_ts_;
            if (zero_vector_duration > MaxZeroVectorDuration)
            {
                return uavcan::protocol::NodeStatus::HEALTH_WARNING;
            }
        }
        else
        {
            last_nonzero_vector_ts_ = uavcan_stm32::SystemClock::instance().getMonotonic();
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
    virtual ~MagThread() { }

    void main() override
    {
        os::watchdog::Timer wdt;
        wdt.startMSec(1000);
        setName("mag");

        ::usleep(500000);         // Startup delay
        wdt.reset();

        node::markComponentInitialized(node::ComponentID::Magnetometer);

        while (!tryInit() && !node::hasPendingRestartRequest())
        {
            setStatus(uavcan::protocol::NodeStatus::HEALTH_ERROR);
            os::lowsyslog("Mag init failed, will retry...\n");
            ::usleep(500000);
            wdt.reset();
        }

        wdt.reset();

        const float variance = param_variance.get();
        const float scaling_coef = param_scaling_coef.get();
        const uint64_t period_usec = param_period_usec.get();

        systime_t sleep_until = chibios_rt::System::getTime();

        while (!node::hasPendingRestartRequest())
        {
            sleep_until += US2ST(period_usec);

            float vector[3] = {0, 0, 0};
            if (tryRead(vector))
            {
                rescale(vector, scaling_coef);
                applyMedianFilter(vector);
                transformToNEDFrame(vector);
                publish(vector, variance);
                setStatus(estimateStatusFromMeasurement(vector));

                os::MutexLocker mlock(last_sample_mutex);
                last_sample.seq_id++;
                std::copy(std::begin(vector), std::end(vector), last_sample.magnetic_field_strength);
            }
            else
            {
                setStatus(uavcan::protocol::NodeStatus::HEALTH_ERROR);
            }

            os::sleepUntilChTime(sleep_until);
            wdt.reset();
        }

        os::lowsyslog("Mag driver terminated\n");
    }
} mag_thread;

}

void init()
{
    (void)mag_thread.start(HIGHPRIO);
}

Sample getLastSample()
{
    os::MutexLocker mlock(last_sample_mutex);
    return last_sample;
}

}
