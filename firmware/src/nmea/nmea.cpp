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

#include <nmea/nmea.hpp>
#include <uavcan/uavcan.hpp>            // Needed for array type
#include <zubax_chibios/sys/sys.h>
#include <unistd.h>

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

    void addEmptyField()
    {
        buffer_ += ",";
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

/**
 * This thread exits automatically if there are no active outputs.
 */
class NMEAOutputThread : public chibios_rt::BaseStaticThread<2048>
{
    static void handleOneOutput(OutputQueue* const out, const char* line)
    {
        chOQWriteTimeout(out, reinterpret_cast<const unsigned char*>(line), std::strlen(line), TIME_IMMEDIATE);
        chOQWriteTimeout(out, reinterpret_cast<const unsigned char*>("\r\n"), 2, TIME_IMMEDIATE);
    }

    msg_t main() override
    {
        setName("nmea");

        for (;;)
        {
            /*
             * Doing nothing as long as there are no outputs
             */
            while (output_registry_.empty())
            {
                ::usleep(1000000);
            }

            ::usleep(100000);

            /*
             * Writing all outputs at once.
             */
            output_registry_.forEach(&NMEAOutputThread::handleOneOutput, "$NMEA,*12");
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
