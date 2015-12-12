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
#include <zubax_chibios/sys/sys.h>
#include <unistd.h>

#include <magnetometer.hpp>
#include <air_sensor.hpp>
#include <gnss.hpp>

namespace nmea
{
namespace
{

struct Locker
{
    Locker(chibios_rt::Mutex& m)
    {
        m.lock();
    }
    ~Locker()
    {
        chibios_rt::BaseThread::unlockMutex();
    }
};


class OutputRegistry
{
    static constexpr unsigned MaxOutputs = 3;
    OutputQueue* outputs_[MaxOutputs] = {};
    unsigned size_ = 0;

    mutable chibios_rt::Mutex mutex_;

public:
    void add(OutputQueue* const out)
    {
        Locker locker(mutex_);
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

    void remove(const OutputQueue* const out)
    {
        Locker locker(mutex_);
        for (auto& x : outputs_)
        {
            if (x == out)
            {
                x = nullptr;
                size_--;
            }
        }
    }

    bool exists(const OutputQueue* const out) const
    {
        Locker locker(mutex_);
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
        Locker locker(mutex_);
        return 0 == size_;
    }

    template <typename Target, typename... Args>
    void forEach(Target target, Args... args)
    {
        Locker locker(mutex_);
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
    static constexpr unsigned MaxSentenceLength = 82;   ///< Explicitly specified

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
        addComplexField("%02u%09.6f", t.deg, t.min);
        addField(t.sign);
    }

    void addLongitude(double deg)
    {
        const auto t = degToDegMinSign(deg, 'E', 'W');
        addComplexField("%03u%09.6f", t.deg, t.min);
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


static void handleOneOutput(OutputQueue* const out, const char* line)
{
    chOQWriteTimeout(out, reinterpret_cast<const unsigned char*>(line), std::strlen(line), TIME_IMMEDIATE);
    chOQWriteTimeout(out, reinterpret_cast<const unsigned char*>("\r\n"), 2, TIME_IMMEDIATE);
}

void outputSentence(SentenceBuilder& b)
{
    output_registry_.forEach(&handleOneOutput, b.compile().c_str());
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

    const auto x = s.magnetic_field_strength[0];
    const auto y = s.magnetic_field_strength[1];

    float heading_deg = std::atan2(y, x) * float(180.0 / M_PI);
    if (heading_deg < 0.f)
    {
        heading_deg += 360.f;
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


void processGNSSFix()
{
    static gnss::Fix fix;
    if (!gnss::getFixIfUpdatedSince(fix.ts.mono_usec, fix))
    {
        return;
    }

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
        const std::time_t unix_time = static_cast<std::time_t>(fix.utc_usec / 1000000U);
        const auto tm = std::gmtime(&unix_time);
        b.addComplexField("%02d%02d%02d.%03u",
                          tm->tm_hour, tm->tm_min, tm->tm_sec,
                          static_cast<unsigned>((fix.utc_usec / 1000U) % 1000U));

        // Status
        const bool valid = fix.utc_valid && (fix.mode >= fix.Mode::Fix3D);
        b.addField(valid ? 'A' : 'V');

        // Lat/Lon
        b.addLatitude(fix.lat);
        b.addLongitude(fix.lon);

        // Speed [knots]
        const float speed = std::sqrt(fix.ned_velocity[0] * fix.ned_velocity[0] +
                                      fix.ned_velocity[1] * fix.ned_velocity[1]);
        b.addField("%06.2f", speed * 1.943844f);

        // Track [degrees]
        float track_deg = std::atan2(fix.ned_velocity[1], fix.ned_velocity[0]) * float(180.0 / M_PI);
        if (track_deg < 0)
        {
            track_deg += 360.0f;
        }
        b.addField("%05.1f", track_deg);

        // Date
        b.addComplexField("%02d%02d%02d",
                          tm->tm_mday, tm->tm_mon + 1, tm->tm_year - 100);

        // Magnetic variation [degrees], east/west
        b.addEmptyField();
        b.addEmptyField();

        outputSentence(b);
    }
}


void processGNSSAux()
{
    static gnss::Auxiliary aux;
    if (!gnss::getAuxiliaryIfUpdatedSince(aux.ts.mono_usec, aux))
    {
        return;
    }
}


class NMEAOutputThread : public chibios_rt::BaseStaticThread<2048>
{
    static constexpr unsigned MinSensorPeriodMSec = 100;

    msg_t main() override
    {
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
            while (output_registry_.empty())
            {
                ::usleep(100000);
                sleep_until = chibios_rt::System::getTime();
            }

            HandlerTable[selector++]();
            if (selector >= NumHandlers)
            {
                selector = 0;
            }

            sleep_until += MS2ST(MinSensorPeriodMSec / NumHandlers);
            sysSleepUntilChTime(sleep_until);
        }

        return 0;
    }
} nmea_output_thread;

}

void init()
{
    nmea_output_thread.start(LOWPRIO + 3);
}

void addOutput(OutputQueue* const output)
{
    output_registry_.add(output);
}

void removeOutput(const OutputQueue* const output)
{
    output_registry_.remove(output);
}

bool hasOutput(const OutputQueue* const output)
{
    return output_registry_.exists(output);
}

}
