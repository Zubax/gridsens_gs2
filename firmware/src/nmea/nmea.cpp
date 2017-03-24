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

#include <cmath>
#include <ctime>
#include <cstdio>
#include <nmea/nmea.hpp>
#include <uavcan/uavcan.hpp>            // Needed for array type
#include <zubax_chibios/os.hpp>
#include <unistd.h>

#include <magnetometer.hpp>
#include <air_sensor.hpp>
#include <gnss.hpp>

namespace nmea
{
namespace
{

static constexpr unsigned MinSatsForGoodFix = 6;

static constexpr unsigned TypicalSentenceLength = 50;
static constexpr unsigned MinRealBaudRate = 115200;                             ///< For physical UART (not USB)
static constexpr unsigned DelayAfterSentenceTransmissionMs = TypicalSentenceLength * 1000 / (MinRealBaudRate / 10);


class OutputRegistry
{
    static constexpr unsigned MaxOutputs = 3;
    ::BaseChannel* outputs_[MaxOutputs] = {};
    unsigned size_ = 0;

    mutable chibios_rt::Mutex mutex_;

public:
    void add(::BaseChannel* const out)
    {
        os::MutexLocker locker(mutex_);
        for (auto& x : outputs_)
        {
            if (x == nullptr)
            {
                x = out;
                size_++;
                break;
            }
        }
    }

    void remove(const ::BaseChannel* const out)
    {
        os::MutexLocker locker(mutex_);
        for (auto& x : outputs_)
        {
            if (x == out)
            {
                x = nullptr;
                size_--;
            }
        }
    }

    bool exists(const ::BaseChannel* const out) const
    {
        os::MutexLocker locker(mutex_);
        for (auto& x : outputs_)
        {
            if (x == out)
            {
                return true;
            }
        }
        return false;
    }

    bool empty() const
    {
        os::MutexLocker locker(mutex_);
        return 0 == size_;
    }

    template <typename Target, typename... Args>
    void forEach(Target target, Args... args)
    {
        os::MutexLocker locker(mutex_);
        for (auto& x : outputs_)
        {
            if (x != nullptr)
            {
                target(x, args...);
            }
        }
    }
} output_registry_;


/**
 * Builds standard NMEA 0183 sentences with checksum.
 */
class SentenceBuilder
{
    /*
     * Sample:
     * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
     */
    static constexpr unsigned MaxSentenceLength = 100;
    typedef typename uavcan::MakeString<MaxSentenceLength>::Type Buffer;

    Buffer buffer_;

    static std::uint8_t computeChecksum(const char* s)
    {
        unsigned c = 0;
        while (*s != 0)
        {
            c ^= *s++;
        }
        return c & 0xFFU;
    }

    struct DegMinSign
    {
        int deg;
        double min;
        char sign;
    };

    DegMinSign degToDegMinSign(double x, char pos, char neg)
    {
        const auto deg = int(x);
        const auto min = 60 * double(x - deg);
        const auto ch = (x >= 0) ? pos : neg;
        return { std::abs(deg), std::abs(min), ch };
    };

public:
    SentenceBuilder(const char* identifier)
    {
        buffer_ += "$";
        buffer_ += identifier;
    }

    template <typename T>
    void addField(const char* const format, const T value)
    {
        buffer_ += ",";
        buffer_.appendFormatted(format, value);
    }

    void addField(const char* const value)
    {
        buffer_ += ",";
        buffer_ += value;
    }

    void addField(char value)
    {
        buffer_ += ",";
        buffer_.push_back(value);
    }

    template <typename... Args>
    void addComplexField(const char* const format, Args... args)
    {
        using namespace std;
        char buf[32];           // Should be enough for any RTCM field
        snprintf(buf, sizeof(buf), format, args...);
        addField(buf);
    }

    void addEmptyField()
    {
        buffer_ += ",";
    }

    void addLatitude(double deg)
    {
        const auto t = degToDegMinSign(deg, 'N', 'S');
        addComplexField("%02u%08.5f", t.deg, t.min);
        addField(t.sign);
    }

    void addLongitude(double deg)
    {
        const auto t = degToDegMinSign(deg, 'E', 'W');
        addComplexField("%03u%08.5f", t.deg, t.min);
        addField(t.sign);
    }

    const Buffer& compile()
    {
        const bool has_crc = (buffer_.size() > 3U) && (buffer_[buffer_.size() - 3U] == '*');
        if (!has_crc && (buffer_.size() > 1))
        {
            const auto checksum = computeChecksum(buffer_.c_str() + 1);
            buffer_.appendFormatted("*%02X", checksum);
        }
        return buffer_;
    }
};


class Decimator
{
    const unsigned ratio_;
    unsigned counter_ = 0;

public:
    Decimator(unsigned ratio) : ratio_(ratio) { }

    bool step()
    {
        if (++counter_ >= ratio_)
        {
            counter_ = 0;
            return true;
        }
        return false;
    }
};


void outputSentence(SentenceBuilder& b)
{
    {
        os::MutexLocker mlock(os::getStdIOMutex());

        static const auto send_one = [](::BaseChannel* const out, const char* line)
        {
            chnWriteTimeout(out, reinterpret_cast<const unsigned char*>(line), std::strlen(line), TIME_IMMEDIATE);
            chnWriteTimeout(out, reinterpret_cast<const unsigned char*>("\r\n"), 2, TIME_IMMEDIATE);
        };

        output_registry_.forEach(send_one, b.compile().c_str());
    }

    /*
     * This delay makes buffer utilization more uniform and predictable.
     * Also, it avoids buffer overruns when sending packages of multiple long messages, e.g. GSV.
     */
    chThdSleepMilliseconds(DelayAfterSentenceTransmissionMs);
}

void processMagnetometer()
{
    static auto seq_id = magnetometer::getLastSample().seq_id;

    auto s = magnetometer::getLastSample();
    if (s.seq_id == seq_id)
    {
        return;
    }
    seq_id = s.seq_id;

    {
        const auto x = s.magnetic_field_strength[0];
        const auto y = s.magnetic_field_strength[1];

        float heading_deg = std::atan2(x, y) * float(180.0 / M_PI);
        heading_deg -= 90.f;
        if (heading_deg < 0.f)
        {
            heading_deg += 360.f;
        }
        if (heading_deg > 360.f)
        {
            heading_deg -= 360.f;
        }

        // http://edu-observatory.org/gps/NMEA_0183.txt
        // http://www.catb.org/gpsd/NMEA.html
        SentenceBuilder b("HCHDG");
        b.addField("%.1f", heading_deg);
        b.addEmptyField();
        b.addEmptyField();
        b.addEmptyField();
        b.addEmptyField();

        outputSentence(b);
    }

    /*
     * Zubax Robotics vendor specific message for magnetic field strength measurement.
     * Format:
     *          $PZUBAX,MAG-FLD-XYZ,1.345,-1.345,0.345,,*12
     */
    {
        SentenceBuilder b("PZUBAX");
        b.addField("MAG-FLD-XYZ");
        for (int i = 0; i < 3; i++)
        {
            b.addComplexField("%.3f", s.magnetic_field_strength[i]);
        }

        b.addField('G');                // Units of measurement - Gauss (SI unit is T, Tesla)
        b.addEmptyField();              // Reserved
        b.addEmptyField();              // Reserved

        outputSentence(b);
    }
}


void processAirSensor()
{
    static auto seq_id = air_sensor::getLastSample().seq_id;

    auto s = air_sensor::getLastSample();
    if (s.seq_id == seq_id)
    {
        return;
    }
    seq_id = s.seq_id;

    const float pressure_bar = s.pressure_pa * 1e-5f;
    const float temperature_degc = s.temperature_k - 273.15f;

    // http://edu-observatory.org/gps/NMEA_0183.txt
    // http://www.catb.org/gpsd/NMEA.html
    {
        SentenceBuilder b("YXXDR");
        b.addField('P');
        b.addField("%.5f", pressure_bar);
        b.addField('B');
        outputSentence(b);
    }

    static Decimator temp_decimator(10);
    if (temp_decimator.step())
    {
        SentenceBuilder b("YXXDR");
        b.addField('C');
        b.addField("%.1f", temperature_degc);
        b.addField('C');
        outputSentence(b);
    }
}


// These structures are shared
static gnss::Auxiliary auxiliary;
static gnss::Fix fix;

void processGNSSAux()
{
    if (!gnss::getAuxiliaryIfUpdatedSince(auxiliary.ts.mono_usec, auxiliary))
    {
        return;
    }

    static bool alternator = false;

    /*
     * $--GSA,a,x,xx,xx,xx,xx,xx,xx,xx,...
     *     | | |  |  |  |  |  |  |
     *     | | |  |  |  |  |  |  +---|
     *     | | |  |  |  |  |  +------|
     *     | | |  |  |  |  +---------|
     *     | | |  |  |  +------------|
     *     | | |  |  +---------------|
     *     | | |  +------------------|PRN numbers of satellites used in
     *     | | +---------------------/solution (null for unused fileds)
     *     | +------------------------Mode: 1 = Fix not available
     *     |                                2 = 2D
     *     |                                3 = 3D
     *     +--------------------------Mode: M = Manual, forced to operate in 2D or 3D mode
     *                                         A = Automatic, allowed to automatically switch 2D/3D
     * xx,xx,xx,xx,xx,x.x,x.x,x.x*hh<CR><LF>
     * |  |  |  |  |   |   |   |
     * |  |  |  |  |   |   |   +------VDOP
     * |  |  |  |  |   |   +----------HDOP
     * |  |  |  |  |   +--------------PDOP
     * |  |  |  |  +-----------------\
     * |  |  |  +--------------------|
     * |  |  +-----------------------|
     * |  +--------------------------|
     * +-----------------------------|
     *
     * $--GSV,x,x,xx,xx,xx,xxx,xx __________ ...
     *     | | |  |  |   |  |      |
     *     | | |  |  |   |  |      |
     *     | | |  |  |   |  |      +-2nd - 3rd SV [2]
     *     | | |  |  |   |  |
     *     | | |  |  |   |  +--------\SNR (C/No) 00-99 dB, null when not tracking
     *     | | |  |  |   +-----------|Azimuth,   degrees True, 000 to 359
     *     | | |  |  +---------------|Elevation, degrees, 90 deg maximum
     *     | | |  +------------------/Satellite PRN number
     *     | | +----------------------Total number of satellites in view
     *     | +------------------------Message number, 1 to 3   [1]
     *     +--------------------------Total number of messages [1]
     * xx,xx,xxx,xx*hh<CR><LF>
     * |  |   |  |
     * |  |   |  +-------------------\
     * |  |   +----------------------|
     * |  +--------------------------|
     * +-----------------------------/4th SV
     */
    if (alternator)
    {
        SentenceBuilder b("GPGSA");

        // Mode
        b.addField('A');
        b.addField("%u", (fix.mode == fix.Mode::Fix2D) ? 2 : ((fix.mode == fix.Mode::Fix3D) ? 3 : 1));

        // Sat numbers
        constexpr unsigned TargetNumber = 12;
        unsigned num_added = 0;
        for (unsigned sat_idx = 0; sat_idx < auxiliary.num_sats; sat_idx++)
        {
            if (auxiliary.sats[sat_idx].used)
            {
                b.addField("%02u", auxiliary.sats[sat_idx].sat_id);
                num_added++;
                if (num_added >= TargetNumber)
                {
                    break;
                }
            }
        }
        while (num_added++ < TargetNumber)
        {
            b.addEmptyField();
        }

        // PDOP, HDOP, VDOP
        b.addField("%.2f", auxiliary.pdop);
        b.addField("%.2f", auxiliary.hdop);
        b.addField("%.2f", auxiliary.vdop);

        outputSentence(b);
    }
    else
    {
        // Counting satellites first
        static const auto is_valid_sv = [](const gnss::Auxiliary::Sat& s)
            {
                return (s.elevation >= 0 && s.elevation <= 90) && (s.signal_noise_ratio > 0);
            };

        unsigned total_num_sats = 0;
        for (unsigned i = 0; i < auxiliary.num_sats; i++)
        {
            if (is_valid_sv(auxiliary.sats[i]))
            {
                total_num_sats++;
            }
        }

        // Now sending the messages
        unsigned sat_index = 0;
        unsigned remaining_sats = total_num_sats;
        const unsigned num_messages = (total_num_sats + 3) / 4;
        for (unsigned msg_num = 1; msg_num <= num_messages; msg_num++)
        {
            SentenceBuilder b("GPGSV");

            // Common fields
            b.addField("%u", num_messages);
            b.addField("%u", msg_num);
            b.addField("%02u", total_num_sats);

            // Per sat info
            unsigned remaining_sats_in_message = 4;
            while (remaining_sats_in_message > 0 && remaining_sats > 0)
            {
                const auto& sat = auxiliary.sats[sat_index++];
                if (!is_valid_sv(sat))
                {
                    continue;
                }

                b.addField("%02u", sat.sat_id);
                b.addField("%02d", sat.elevation);
                b.addField("%03d", sat.azimuth);
                b.addField("%02d", sat.signal_noise_ratio);

                remaining_sats_in_message--;
                remaining_sats--;
            }

            outputSentence(b);
        }
    }

    alternator = !alternator;
}


void processGNSSFix()
{
    if (!gnss::getFixIfUpdatedSince(fix.ts.mono_usec, fix))
    {
        return;
    }

    /*
     * Common values
     */
    const std::time_t unix_time = static_cast<std::time_t>(fix.utc_usec / 1000000U);
    const auto tm = std::gmtime(&unix_time);

    const bool fix_valid = fix.utc_valid && (fix.mode >= fix.Mode::Fix3D) && (fix.sats_used >= MinSatsForGoodFix);

    /*
     * http://edu-observatory.org/gps/NMEA_0183.txt
     * http://www.catb.org/gpsd/NMEA.html
     *
     * $--RMC,hhmmss.ss,A,llll.ll,a,...
     *         |       |   |     |
     *         |       |   |     +-----\N/S North or South
     *         |       |   +-----------/Latitude
     *         |       +----------------Status: V = Nav. receiver warning
     *         +------------------------UTC of position fix
     *
     * yyyyy.yy,a,x.x,x.x,xxxxxx,...
     *     |      |  |   |    |
     *     |      |  |   |    +---------Date: dd|mm|yy
     *     |      |  |   +--------------Track made good, degrees True
     *     |      |  +------------------Speed over ground, knots
     *     |      +--------------------\E/W East or West
     *     +---------------------------/Longitude
     *
     * x.x,a*hh<CR><LF>
     *     |  | |
     *     |  | +------------------------Checksum, mandatory for RMC
     *     |  +-------------------------\E/W East or West [1]
     *     +----------------------------/Magnetic variation, degrees
     */
    {
        SentenceBuilder b("GPRMC");

        // Time
        b.addComplexField("%02d%02d%02d.%02u",
                          tm->tm_hour, tm->tm_min, tm->tm_sec,
                          static_cast<unsigned>((fix.utc_usec / 10000U) % 100U));

        // Status
        b.addField(fix_valid ? 'A' : 'V');

        // Lat/Lon
        b.addLatitude(fix.lat);
        b.addLongitude(fix.lon);

        // Speed [knots]
        const float speed = std::sqrt(fix.ned_velocity[0] * fix.ned_velocity[0] +
                                      fix.ned_velocity[1] * fix.ned_velocity[1]);
        b.addField("%.3f", speed * 1.943844f);

        // Track [degrees]
        b.addField("%05.1f", fix.heading_of_motion);

        // Date
        b.addComplexField("%02d%02d%02d",
                          tm->tm_mday, tm->tm_mon + 1, std::max(tm->tm_year - 100, 0));

        // Magnetic variation [degrees], east/west
        b.addEmptyField();
        b.addEmptyField();

        outputSentence(b);
    }

    /*
     * $--GGA,hhmmss.ss,llll.ll,a,...
     *             |      |     |
     *             |      |     +-------\N/S North or South
     *             |      +-------------/Latitude
     *             +---------------------UTC of position
     *
     * yyyyy.yy,a,x,xx,x.x,x.x,M,...
     *     |      | | |   |   |  |
     *     |      | | |   |   |  +-----\Units of antenna altitude, meters
     *     |      | | |   |   +--------/Antenna altitude above/below mean-sea-level (geoid)
     *     |      | | |   +-------------Horizontal dilution of precision
     *     |      | | +-----------------Number of satellites in use, 00-12,
     *     |      | |                   may be different from the number in view
     *     |      | +-------------------GPS quality indicator [1]
     *     |      +--------------------\E/W East or West
     *     +---------------------------/Longitude
     *
     * x.x,M,x.x,xxxx*hh<CR><LF>
     *     |  |  |   |
     *     |  |  |   +-------------------Differential reference station ID, 0000-1023
     *     |  |  +-----------------------Age of Differential GPS data [2]
     *     |  +-------------------------\Units of geoidal seperation, meters
     *     +----------------------------/Geoidal seperation [3]
     */
    {
        SentenceBuilder b("GPGGA");

        // Time
        b.addComplexField("%02d%02d%02d.%02u",
                          tm->tm_hour, tm->tm_min, tm->tm_sec,
                          static_cast<unsigned>((fix.utc_usec / 10000U) % 100U));

        // Lat/Lon
        b.addLatitude(fix.lat);
        b.addLongitude(fix.lon);

        // Quality indicator [0 - no fix, 1 - valid fix, 2 - differential fix]
        b.addField("%u", fix_valid ? 1 : 0);

        // Sats in use
        b.addField("%02u", fix.sats_used);

        // HDOP (if HDOP is not initialized yet, use PDOP)
        b.addField("%.2f", (auxiliary.hdop > 1e-9f) ? auxiliary.hdop : fix.pdop);

        // Altitude [meters]
        b.addField("%.3f", fix.height_amsl);
        b.addField('M');

        // Geoidal separation
        b.addField("%.1f", fix.height_wgs84 - fix.height_amsl);
        b.addField('M');

        // Differential data
        b.addEmptyField();
        b.addEmptyField();

        outputSentence(b);
    }
}


class NMEAOutputThread : public chibios_rt::BaseStaticThread<2048>
{
    static constexpr unsigned MinSensorPeriodMSec = 100;

    void main() override
    {
        os::watchdog::Timer wdt;
        wdt.startMSec(1000);

        setName("nmea");

        systime_t sleep_until = chibios_rt::System::getTime();

        static constexpr void(*HandlerTable[])() =
        {
            processMagnetometer,
            processAirSensor,
            processGNSSFix,
            processGNSSAux
        };
        static constexpr unsigned NumHandlers = sizeof(HandlerTable) / sizeof(HandlerTable[0]);

        unsigned selector = 0;

        for (;;)
        {
            wdt.reset();

            while (output_registry_.empty())
            {
                ::usleep(100000);
                sleep_until = chibios_rt::System::getTime();
                wdt.reset();
            }

            HandlerTable[selector++]();
            if (selector >= NumHandlers)
            {
                selector = 0;
            }

            sleep_until += MS2ST(MinSensorPeriodMSec / NumHandlers);
            os::sleepUntilChTime(sleep_until);
        }
    }

public:
    virtual ~NMEAOutputThread() { }
} nmea_output_thread;

}

void init()
{
    nmea_output_thread.start(LOWPRIO + 3);
}

void addOutput(::BaseChannel* const output)
{
    output_registry_.add(output);
}

void removeOutput(const ::BaseChannel* const output)
{
    output_registry_.remove(output);
}

bool hasOutput(const ::BaseChannel* const output)
{
    return output_registry_.exists(output);
}

}
