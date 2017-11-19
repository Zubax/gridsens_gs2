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

#include "gnss.hpp"
#include "node.hpp"

#include <uavcan/equipment/gnss/RTCMStream.hpp>
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Fix2.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>

#include <ch.hpp>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/util/helpers.hpp>
#include <unistd.h>


namespace gnss
{
namespace
{

SerialDriver* const serial_port = &SD2;

os::config::Param<unsigned> param_gnss_fix_period_usec("uavcan.pubp-fix",
                                                       100000, 66666, 2000000);

os::config::Param<unsigned> param_gnss_aux_period_usec("uavcan.pubp-aux",
                                                       1000000, 100000, 1000000);

os::config::Param<unsigned> param_gnss_fix_prio("uavcan.prio-fix",
                                                16,
                                                uavcan::TransferPriority::NumericallyMin,
                                                uavcan::TransferPriority::NumericallyMax);

os::config::Param<unsigned> param_gnss_aux_prio("uavcan.prio-aux",
                                                20,
                                                uavcan::TransferPriority::NumericallyMin,
                                                uavcan::TransferPriority::NumericallyMax);

os::config::Param<unsigned> param_gnss_warn_min_fix_dimensions("gnss.warn_dimens", 0, 0, 3);
os::config::Param<unsigned> param_gnss_warn_min_sats_used("gnss.warn_sats", 0, 0, 20);

os::config::Param<std::uint8_t> param_gnss_dynamic_model("gnss.dyn_model",
                                                         std::uint8_t(ublox::Config().dynamic_model),
                                                         0,
                                                         ublox::Config::NumDynamicModels - 1);

os::config::Param<bool> param_gnss_use_old_fix_message("gnss.old_fix_msg", true);

chibios_rt::Mutex last_sample_mutex;
Auxiliary last_sample_aux;
Fix last_sample_fix;


std::uint16_t computeNumLeapSecondsFromGpsLeapSeconds(std::uint16_t gps_leaps)
{
    return gps_leaps + 9;
}

void publishFix2(const Fix& data, const ublox::GpsLeapSeconds& leaps)
{
    uavcan::equipment::gnss::Fix2 msg;

    // Timestamp - Network clock
    msg.timestamp = uavcan::UtcTime::fromUSec(data.ts.real_usec);

    // Timestamp - GNSS clock
    msg.gnss_timestamp = data.utc_valid ? uavcan::UtcTime::fromUSec(data.utc_usec) : uavcan::UtcTime();

    msg.gnss_time_standard = uavcan::equipment::gnss::Fix2::GNSS_TIME_STANDARD_UTC;

    if (leaps.num_leap_seconds > 0)     // Zero means unknown
    {
        msg.num_leap_seconds = computeNumLeapSecondsFromGpsLeapSeconds(leaps.num_leap_seconds);
    }

    // Position LLA
    msg.latitude_deg_1e8  = static_cast<std::int64_t>(data.lat * 1e8);
    msg.longitude_deg_1e8 = static_cast<std::int64_t>(data.lon * 1e8);

    msg.height_ellipsoid_mm = static_cast<std::int32_t>(data.height_wgs84 * 1e3F);
    msg.height_msl_mm       = static_cast<std::int32_t>(data.height_amsl * 1e3F);

    // Velocity
    std::copy(std::begin(data.ned_velocity), std::end(data.ned_velocity), std::begin(msg.ned_velocity));

    // Mode
    if (data.mode == Fix::Mode::Time)
    {
        msg.status = uavcan::equipment::gnss::Fix2::STATUS_TIME_ONLY;
    }
    else if (data.mode == Fix::Mode::Fix2D)
    {
        msg.status = uavcan::equipment::gnss::Fix2::STATUS_2D_FIX;
    }
    else if (data.mode == Fix::Mode::Fix3D)
    {
        msg.status = uavcan::equipment::gnss::Fix2::STATUS_3D_FIX;
    }
    else
    {
        msg.status = uavcan::equipment::gnss::Fix2::STATUS_NO_FIX;
    }

    msg.mode = ((data.flags & Fix::Flags::DifferentialSolution) != 0) ?
        uavcan::equipment::gnss::Fix2::MODE_DGPS :
        uavcan::equipment::gnss::Fix2::MODE_SINGLE;

    // Uncertainty
    if (msg.status > uavcan::equipment::gnss::Fix2::STATUS_TIME_ONLY)
    {
        // Assuming that the matrices are diagonal.
        // Full construction and compression of a 6x6 matrix is too computationally expensive for this MCU at 15 Hz.
        // Position
        msg.covariance.push_back(data.position_covariance[0]);
        msg.covariance.push_back(data.position_covariance[4]);
        msg.covariance.push_back(data.position_covariance[8]);
        // Velocity
        msg.covariance.push_back(data.velocity_covariance[0]);
        msg.covariance.push_back(data.velocity_covariance[4]);
        msg.covariance.push_back(data.velocity_covariance[8]);
    }

    // Misc
    msg.sats_used = data.sats_used;
    msg.pdop = data.pdop;

    // ECEF position velocity
    uavcan::equipment::gnss::ECEFPositionVelocity ecef;

    for (int i = 0; i < 3; i++)
    {
        ecef.position_xyz_mm[i] = static_cast<std::int64_t>(data.ecef.position[i] * 1e3);
        ecef.velocity_xyz[i] = data.ecef.velocity[i];
    }

    // ECEF covariance (always diagonal)
    ecef.covariance.push_back(data.ecef.position_variance[0]);
    ecef.covariance.push_back(data.ecef.position_variance[1]);
    ecef.covariance.push_back(data.ecef.position_variance[2]);
    ecef.covariance.push_back(data.ecef.velocity_variance[0]);
    ecef.covariance.push_back(data.ecef.velocity_variance[1]);
    ecef.covariance.push_back(data.ecef.velocity_variance[2]);

    msg.ecef_position_velocity.push_back(ecef);

    // Publishing
    node::Lock locker;
    static uavcan::Publisher<uavcan::equipment::gnss::Fix2> pub(node::getNode());

    EXECUTE_ONCE_NON_THREAD_SAFE
    {
        pub.setPriority(param_gnss_fix_prio.get());
    }

    (void)pub.broadcast(msg);
}

void publishOldFix(const Fix& data, const ublox::GpsLeapSeconds& leaps)
{
    uavcan::equipment::gnss::Fix msg;

    // Timestamp - Network clock
    msg.timestamp = uavcan::UtcTime::fromUSec(data.ts.real_usec);

    // Timestamp - GNSS clock
    msg.gnss_timestamp = data.utc_valid ? uavcan::UtcTime::fromUSec(data.utc_usec) : uavcan::UtcTime();

    msg.gnss_time_standard = uavcan::equipment::gnss::Fix::GNSS_TIME_STANDARD_UTC;

    if (leaps.num_leap_seconds > 0)     // Zero means unknown
    {
        msg.num_leap_seconds = computeNumLeapSecondsFromGpsLeapSeconds(leaps.num_leap_seconds);
    }

    // Position
    msg.latitude_deg_1e8  = static_cast<std::int64_t>(data.lat * 1e8);
    msg.longitude_deg_1e8 = static_cast<std::int64_t>(data.lon * 1e8);

    msg.height_ellipsoid_mm = static_cast<std::int32_t>(data.height_wgs84 * 1e3F);
    msg.height_msl_mm       = static_cast<std::int32_t>(data.height_amsl * 1e3F);

    // Velocity
    std::copy(std::begin(data.ned_velocity), std::end(data.ned_velocity), std::begin(msg.ned_velocity));

    // Mode
    if (data.mode == Fix::Mode::Time)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY;
    }
    else if (data.mode == Fix::Mode::Fix2D)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_2D_FIX;
    }
    else if (data.mode == Fix::Mode::Fix3D)
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_3D_FIX;
    }
    else
    {
        msg.status = uavcan::equipment::gnss::Fix::STATUS_NO_FIX;
    }

    // Uncertainty
    if (msg.status > uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY)
    {
        msg.velocity_covariance.packSquareMatrix(data.velocity_covariance);
        msg.position_covariance.packSquareMatrix(data.position_covariance);
    }

    // Misc
    msg.sats_used = data.sats_used;
    msg.pdop = data.pdop;

    // Publishing
    node::Lock locker;
    static uavcan::Publisher<uavcan::equipment::gnss::Fix> pub(node::getNode());

    EXECUTE_ONCE_NON_THREAD_SAFE
    {
        // One lower than the requested priority; this is needed in order to give the new Fix message higher priority
        pub.setPriority(std::min<std::uint8_t>(uavcan::TransferPriority::NumericallyMax,
                                               param_gnss_fix_prio.get() + 1));
    }

    (void)pub.broadcast(msg);
}

void publishAuxiliary(const Fix& fix, const Auxiliary& aux)
{
    uavcan::equipment::gnss::Auxiliary msg;

    // DOP
    msg.gdop = aux.gdop;
    msg.hdop = aux.hdop;
    msg.pdop = aux.pdop;
    msg.tdop = aux.tdop;
    msg.vdop = aux.vdop;
    msg.ndop = aux.ndop;
    msg.edop = aux.edop;

    // Satellite stats
    msg.sats_used = fix.sats_used;
    msg.sats_visible = aux.num_sats;

    // Publishing
    node::Lock locker;
    static uavcan::Publisher<uavcan::equipment::gnss::Auxiliary> pub(node::getNode());

    EXECUTE_ONCE_NON_THREAD_SAFE
    {
        pub.setPriority(param_gnss_aux_prio.get());
    }

    (void)pub.broadcast(msg);
}


class Platform : public ublox::IPlatform
{
public:
    void portWrite(const std::uint8_t* data, unsigned len) override
    {
        sdWrite(serial_port, data, len);
    }

    unsigned portRead(std::uint8_t* out_data, unsigned max_len, unsigned timeout_ms) override
    {
        return sdReadTimeout(serial_port, out_data, max_len, MS2ST(timeout_ms));
    }

    void portSetBaudRate(unsigned new_baudrate) override
    {
        static SerialConfig serial_cfg;
        serial_cfg = SerialConfig();
        serial_cfg.speed = new_baudrate;
        serial_cfg.cr2 = USART_CR2_STOP1_BITS;

        sdStop(serial_port);
        sdStart(serial_port, &serial_cfg);
    }

    std::uint64_t getMonotonicUSec() const override
    {
        return uavcan_stm32::SystemClock::instance().getMonotonic().toUSec();
    }

    std::uint64_t getRealUSec() const override
    {
        return uavcan_stm32::SystemClock::instance().getUtc().toUSec();
    }
};


class GnssThread : public chibios_rt::BaseStaticThread<3000>
{
    bool keep_going_ = true;

    unsigned warn_min_fix_dimensions_ = 0;
    unsigned warn_min_sats_used_ = 0;
    bool use_old_fix_message_ = true;

    mutable os::watchdog::Timer watchdog_;
    mutable Platform platform_;
    mutable ublox::Driver driver_ = ublox::Driver(platform_);

    bool shouldKeepGoing() const
    {
        return keep_going_ && !os::isRebootRequested();
    }

    void pauseOneSec() const
    {
        watchdog_.reset();
        ::usleep(500000);
        watchdog_.reset();
        ::usleep(500000);
        watchdog_.reset();
    }

    void tryInit() const
    {
        auto cfg = ublox::Config();
        cfg.fix_rate_hz = 1e6F / param_gnss_fix_period_usec.get();
        cfg.aux_rate_hz = 1e6F / param_gnss_aux_period_usec.get();
        cfg.dynamic_model = ublox::Config::DynamicModel(param_gnss_dynamic_model.get());

        while (shouldKeepGoing() && !driver_.configure(cfg, watchdog_))
        {
            os::lowsyslog("GNSS driver init failed\n");
            pauseOneSec();
        }
    }

    void tryRun() const
    {
        do
        {
            watchdog_.reset();
            driver_.spin(100);
        }
        while (shouldKeepGoing() && driver_.areRatesValid());
    }

    void handleFix(const Fix& fix) const
    {
        // Publish the new GNSS solution onto the bus
        publishFix2(fix, driver_.getGpsLeapSeconds());
        if (use_old_fix_message_)
        {
            publishOldFix(fix, driver_.getGpsLeapSeconds());
        }

        // Update component status
        const bool warn = (static_cast<unsigned>(fix.mode) < warn_min_fix_dimensions_) ||
                          (fix.sats_used < warn_min_sats_used_);
        auto stat = warn ? uavcan::protocol::NodeStatus::HEALTH_WARNING : uavcan::protocol::NodeStatus::HEALTH_OK;
        node::setComponentHealth(node::ComponentID::Gnss, stat);

        // Adjust the local time (locked to the global UTC)
        if (fix.utc_valid)
        {
            auto adj = uavcan::UtcTime::fromUSec(fix.utc_usec) - uavcan::UtcTime::fromUSec(fix.ts.real_usec);
            node::adjustUtcTimeFromLocalSource(adj);
        }

        // This is very slow, updating in the last order in order to not delay the data
        os::MutexLocker mlock(last_sample_mutex);
        last_sample_fix = fix;
    }

public:
    virtual ~GnssThread() { }

    void main() override
    {
        watchdog_.startMSec(1000);
        setName("gnss");

        warn_min_fix_dimensions_ = param_gnss_warn_min_fix_dimensions.get();
        warn_min_sats_used_ = param_gnss_warn_min_sats_used.get();
        use_old_fix_message_ = param_gnss_use_old_fix_message.get();

        driver_.on_fix = std::bind(&GnssThread::handleFix, this, std::placeholders::_1);
        driver_.on_aux = [this](const Auxiliary& aux)
            {
                publishAuxiliary(driver_.getFix(), aux);
                // This is very slow, updating in the last order in order to not delay the data
                os::MutexLocker mlock(last_sample_mutex);
                last_sample_aux = aux;
            };

        pauseOneSec();  // Waiting for the receiver to boot

        node::markComponentInitialized(node::ComponentID::Gnss);

        while (shouldKeepGoing())
        {
            pauseOneSec();
            os::lowsyslog("GNSS init...\n");
            tryInit();
            tryRun();
            node::setComponentHealth(node::ComponentID::Gnss, uavcan::protocol::NodeStatus::HEALTH_ERROR);
        }

        os::lowsyslog("GNSS driver terminated\n");
        while (true)
        {
            pauseOneSec();   // We don't exit the thread because GNSS driver termination is not fatal
        }
    }

    void stop() override
    {
        keep_going_ = false;
    }
} gnss_thread;

}

void init()
{
    (void)gnss_thread.start(HIGHPRIO - 5);
}

void stop()
{
    gnss_thread.stop();
}

SerialDriver& getSerialPort()
{
    return *serial_port;
}

bool getAuxiliaryIfUpdatedSince(std::uint64_t ts_mono_usec, Auxiliary& out_aux)
{
    os::MutexLocker mlock(last_sample_mutex);
    if (last_sample_aux.ts.mono_usec > ts_mono_usec)
    {
        out_aux = last_sample_aux;
        return true;
    }
    return false;
}

bool getFixIfUpdatedSince(std::uint64_t ts_mono_usec, Fix& out_fix)
{
    os::MutexLocker mlock(last_sample_mutex);
    if (last_sample_fix.ts.mono_usec > ts_mono_usec)
    {
        out_fix = last_sample_fix;
        return true;
    }
    return false;
}

}
