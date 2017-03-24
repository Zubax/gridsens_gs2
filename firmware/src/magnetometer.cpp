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
#include <zubax_chibios/util/helpers.hpp>
#include <unistd.h>


namespace magnetometer
{
namespace
{

static auto& SPID = SPID3;

const float AbsMaxValidGauss = 12.0F;                                           ///< For the selected gain
const auto MaxZeroVectorDuration = uavcan::MonotonicDuration::fromMSec(5000);   ///< Should be OK

const float GaussScale = 0.00043840420868040335F;

os::config::Param<float> param_scaling_coef("mag.scaling_coef", 1.0F, 0.1F, 2.0F);

os::config::Param<bool> param_power_on_self_test("mag.pwron_slftst", true);

os::config::Param<float> param_variance("mag.variance", 0.005, 1e-6, 1.0);

os::config::Param<unsigned> param_period_usec("uavcan.pubp-mag", 10000, 6666, 1000000);

os::config::Param<unsigned> param_prio("uavcan.prio-mag",
                                       16,
                                       uavcan::TransferPriority::NumericallyMin,
                                       uavcan::TransferPriority::NumericallyMax);

chibios_rt::Mutex last_sample_mutex;
Sample last_sample;

namespace lis3mdl
{

constexpr std::uint8_t WHO_AM_I     = 0x0F;
constexpr std::uint8_t CTRL_REG1    = 0x20;
constexpr std::uint8_t CTRL_REG2    = 0x21;
constexpr std::uint8_t CTRL_REG3    = 0x22;
constexpr std::uint8_t CTRL_REG4    = 0x23;
constexpr std::uint8_t CTRL_REG5    = 0x24;
constexpr std::uint8_t STATUS_REG   = 0x27;
constexpr std::uint8_t OUT_X_L      = 0x28;
constexpr std::uint8_t OUT_X_H      = 0x29;
constexpr std::uint8_t OUT_Y_L      = 0x2A;
constexpr std::uint8_t OUT_Y_H      = 0x2B;
constexpr std::uint8_t OUT_Z_L      = 0x2C;
constexpr std::uint8_t OUT_Z_H      = 0x2D;
constexpr std::uint8_t TEMP_OUT_L   = 0x2E;
constexpr std::uint8_t TEMP_OUT_H   = 0x2F;

namespace bit
{

constexpr std::uint8_t ZYXDA    = 3;
constexpr std::uint8_t BDU      = 6;

}
}

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

template <unsigned IOSize>
std::array<uint8_t, IOSize> io(const std::array<uint8_t, IOSize>& tx)
{
    spiAcquireBus(&SPID);
    palClearPad(GPIO_PORT_COMPASS_CHIP_SELECT, GPIO_PIN_COMPASS_CHIP_SELECT);

    std::array<uint8_t, IOSize> rx;
    spiExchange(&SPID, tx.size(), tx.data(), rx.data());

    palSetPad(GPIO_PORT_COMPASS_CHIP_SELECT, GPIO_PIN_COMPASS_CHIP_SELECT);
    spiReleaseBus(&SPID);

    return rx;
}

template <unsigned TxSize>
void write(const std::uint8_t address, const std::array<uint8_t, TxSize>& tx)
{
    static_assert(TxSize > 0, "TxSize");
    assert(address <= 0b00111111);

    std::array<uint8_t, TxSize + 1> write_buf;
    write_buf[0] = address | 0b01000000;                        // Set MS bit (memory increment)
    std::copy(tx.begin(), tx.end(), write_buf.begin() + 1);

    (void) io(write_buf);
}

void write(const std::uint8_t address, const std::uint8_t byte)
{
    const std::array<uint8_t, 1> tx{ byte };
    write(address, tx);
}

template <unsigned RxSize>
std::array<uint8_t, RxSize> read(const std::uint8_t address)
{
    static_assert(RxSize > 0, "RxSize");
    assert(address <= 0b00111111);

    std::array<uint8_t, RxSize + 1> io_buf;
    std::fill(io_buf.begin(), io_buf.end(), 0);
    io_buf[0] = address | 0b11000000;                           // Set MS bit (memory increment) and RW bit (read)

    io_buf = io(io_buf);

    std::array<uint8_t, RxSize> out;
    std::copy(io_buf.begin() + 1, io_buf.end(), out.begin());

    return out;
}

std::uint8_t readByte(const std::uint8_t address)
{
    return read<1>(address)[0];
}

bool writeWithCheck(const std::uint8_t address, const std::uint8_t byte)
{
    write(address, byte);
    const std::uint8_t readback = readByte(address);
    if (readback == byte)
    {
        return true;
    }
    else
    {
        ::os::lowsyslog("Mag: Register write failure: address 0x%02x, wrote 0x%02x, got 0x%02x\n",
                        address, byte, readback);
        return false;
    }
}

void readMagneticFieldStrength(float out_gauss[3])
{
    const auto rx = read<6>(lis3mdl::OUT_X_L);

    const std::int16_t raw_xyz[3]
    {
        std::int16_t(std::uint16_t(rx[0]) | std::uint16_t(std::uint16_t(rx[1]) << 8)),
        std::int16_t(std::uint16_t(rx[2]) | std::uint16_t(std::uint16_t(rx[3]) << 8)),
        std::int16_t(std::uint16_t(rx[4]) | std::uint16_t(std::uint16_t(rx[5]) << 8))
    };

    out_gauss[0] = raw_xyz[0] * GaussScale;
    out_gauss[1] = raw_xyz[1] * GaussScale;
    out_gauss[2] = raw_xyz[2] * GaussScale;
}

/**
 * Refer to AN4602 for explanation.
 */
bool performSelfTest()
{
    // Initial setup
    write(lis3mdl::CTRL_REG1, 0x1C);
    write(lis3mdl::CTRL_REG2, 0x40);
    write(lis3mdl::CTRL_REG5, lis3mdl::bit::BDU);       // Block mode update
    usleep(20000);
    write(lis3mdl::CTRL_REG3, 0x00);
    usleep(20000);

    // Check register configuration
    {
        if (readByte(lis3mdl::CTRL_REG1) != 0x1C)
        {
            os::lowsyslog("Mag: CTRL_REG1 mismatch\n");
            return false;
        }

        if (readByte(lis3mdl::CTRL_REG2) != 0x40)
        {
            os::lowsyslog("Mag: CTRL_REG2 mismatch\n");
            return false;
        }

        if (readByte(lis3mdl::CTRL_REG3) != 0x00)
        {
            os::lowsyslog("Mag: CTRL_REG3 mismatch\n");
            return false;
        }
    }

    // Discard first sample
    {
        usleep(20000);
        const std::uint8_t status_reg = readByte(lis3mdl::STATUS_REG);
        if ((status_reg & (1 << lis3mdl::bit::ZYXDA)) == 0)
        {
            os::lowsyslog("Mag: ZYXDA not set (STATUS_REG %02x)\n", status_reg);
            return false;
        }
        float dummy[3]{};
        readMagneticFieldStrength(dummy);
    }

    // Averaging 5x
    float average[3]{};
    for (int i = 0; i < 5; i++)
    {
        usleep(20000);
        if ((readByte(lis3mdl::STATUS_REG) & (1 << lis3mdl::bit::ZYXDA)) == 0)
        {
            os::lowsyslog("Mag: ZYXDA not set\n");
            return false;
        }

        float sample[3]{};
        readMagneticFieldStrength(sample);
        average[0] += sample[0];
        average[1] += sample[1];
        average[2] += sample[2];
    }
    average[0] /= 5.0F;
    average[1] /= 5.0F;
    average[2] /= 5.0F;
    os::lowsyslog("Mag: AVG %f %f %f G\n", average[0], average[1], average[2]);

    // Enable self test
    write(lis3mdl::CTRL_REG1, 0x1D);
    usleep(60000);

    // Discard first sample
    {
        usleep(20000);
        if ((readByte(lis3mdl::STATUS_REG) & (1 << lis3mdl::bit::ZYXDA)) == 0)
        {
            os::lowsyslog("Mag: ZYXDA not set\n");
            return false;
        }
        float dummy[3]{};
        readMagneticFieldStrength(dummy);
    }

    // Averaging 5x
    float average_self_test[3]{};
    for (int i = 0; i < 5; i++)
    {
        usleep(20000);
        if ((readByte(lis3mdl::STATUS_REG) & (1 << lis3mdl::bit::ZYXDA)) == 0)
        {
            os::lowsyslog("Mag: ZYXDA not set\n");
            return false;
        }

        float sample[3]{};
        readMagneticFieldStrength(sample);
        average_self_test[0] += sample[0];
        average_self_test[1] += sample[1];
        average_self_test[2] += sample[2];
    }
    average_self_test[0] /= 5.0F;
    average_self_test[1] /= 5.0F;
    average_self_test[2] /= 5.0F;
    os::lowsyslog("Mag: AVGST %f %f %f G\n", average_self_test[0], average_self_test[1], average_self_test[2]);

    // Validation
    {
        const float dx = std::abs(average[0] - average_self_test[0]);
        const float dy = std::abs(average[1] - average_self_test[1]);
        const float dz = std::abs(average[2] - average_self_test[2]);
        // See the datasheet for thresholds
        const bool ok =
            (0.90F <= dx) && (dx <= 3.1F) &&
            (0.90F <= dy) && (dy <= 3.1F) &&
            (0.05F <= dz) && (dz <= 1.1F);

        os::lowsyslog("Mag: ST dxyz: %f %f %f G\n", dx, dy, dz);

        if (!ok)
        {
            return false;
        }
    }

    // Disable self test
    if (!writeWithCheck(lis3mdl::CTRL_REG1, 0x1C))
    {
        os::lowsyslog("Mag: Could not disable self test\n");
        return false;
    }

    if (!writeWithCheck(lis3mdl::CTRL_REG3, 0x03))
    {
        os::lowsyslog("Mag: Could not power down\n");
        return false;
    }

    return true;
}

bool tryInit()
{
    /*
     * Connectivity check
     */
    {
        const std::uint8_t who_am_i = readByte(lis3mdl::WHO_AM_I);
        if (who_am_i != 0b00111101)
        {
            os::lowsyslog("Mag: WHO_AM_I mismatch: %02x\n", who_am_i);
            return false;
        }
    }

    /*
     * Run self test
     */
    if (param_power_on_self_test)
    {
        if (!performSelfTest())
        {
            ::os::lowsyslog("Mag self test failed\n");
            return false;
        }

        ::os::lowsyslog("Mag self test OK\n");
    }
    else
    {
        ::os::lowsyslog("Mag self test skipped - disabled by configuration\n");
    }

    /*
     * Configure
     */
    if (!writeWithCheck(lis3mdl::CTRL_REG3, 0b00000011))        // Disable sensor during configuration
    {
        return false;
    }

    if (!writeWithCheck(lis3mdl::CTRL_REG1, 0b11111110))        // Ultra high performance, 155 Hz update rate
    {
        return false;
    }

    if (!writeWithCheck(lis3mdl::CTRL_REG2, 0b01000000))        // 12 Gauss range
    {
        return false;
    }

    if (!writeWithCheck(lis3mdl::CTRL_REG4, 0b00001100))        // Z axis configuration should match X and Y
    {
        return false;
    }

    if (!writeWithCheck(lis3mdl::CTRL_REG5, 0b01000000))        // Block data update
    {
        return false;
    }

    if (!writeWithCheck(lis3mdl::CTRL_REG3, 0b00000000))        // Enter continuous conversion mode
    {
        return false;
    }

    /*
     * Discard the first couple of samples after initialization
     */
    float dummy[3]{};
    usleep(20000);
    readMagneticFieldStrength(dummy);
    usleep(20000);
    readMagneticFieldStrength(dummy);

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
    /*
     * Board frame is in the NED frame which is standard in aerospace:
     *  X - along the FORWARD arrow
     *  Y - to the right of the FORWARD arrow when looking from the TOP side
     *  Z - towards the BOTTOM side
     */
    const float x =  inout_mag_vector[1];
    const float y = -inout_mag_vector[0];
    const float z =  inout_mag_vector[2];

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

        while (!tryInit() && !os::isRebootRequested())
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

        while (!os::isRebootRequested())
        {
            sleep_until += US2ST(period_usec);

            {
                float vector[3] = {0, 0, 0};
                readMagneticFieldStrength(vector);

                rescale(vector, scaling_coef);
                transformToNEDFrame(vector);
                publish(vector, variance);
                setStatus(estimateStatusFromMeasurement(vector));

                os::MutexLocker mlock(last_sample_mutex);
                last_sample.seq_id++;
                std::copy(std::begin(vector), std::end(vector), last_sample.magnetic_field_strength);
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
