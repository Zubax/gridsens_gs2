/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#pragma once

#include "ublox_msg.hpp"
#include <cassert>
#include <cstdint>
#include <functional>
#include <algorithm>
#include <crdr_chibios/watchdog/watchdog.hpp>

namespace ublox
{

struct Message
{
    std::uint8_t cls = 0;
    std::uint8_t id = 0;
    const std::uint8_t* payload = nullptr;
    std::uint16_t len = 0;

    template <typename MessageStruct>
    static Message make(const MessageStruct& msgstruct, unsigned len = sizeof(MessageStruct))
    {
        auto message = Message();
        message.cls = MessageStruct::Class;
        message.id  = MessageStruct::ID;
        message.len = len;
        message.payload = reinterpret_cast<const std::uint8_t*>(&msgstruct);
        return message;
    }
};

struct Timestamps
{
    std::uint64_t mono_usec = 0;
    std::uint64_t real_usec = 0;
};

struct RxMessage : public Message, public Timestamps
{
    template <typename PayloadStruct>
    const PayloadStruct* tryCastTo() const
    {
        if (cls == PayloadStruct::Class && id == PayloadStruct::ID)
        {
            return reinterpret_cast<const PayloadStruct*>(payload);
        }
        return nullptr;
    }
};

/*
 * Output data structures
 */
struct Fix
{
    enum class Mode
    {
        None,
        Time,
        Fix2D,
        Fix3D
    };

    struct Flags
    {
        static constexpr unsigned DifferentialSolution = 1;
    };

    Timestamps ts;
    std::uint64_t utc_usec;
    bool utc_valid;
    double lat;                   ///< deg
    double lon;                   ///< deg
    float alt;                    ///< meters
    float ned_velocity[3];        ///< North-east-down, m/s
    float pdop;
    float position_covariance[9]; ///< Lat-lon-alt, row major, m^2
    float velocity_covariance[9]; ///< North-east-down, row major, (m/s)^2
    Mode mode;
    unsigned flags;
    unsigned sats_used;
};

struct Aux
{
    struct Sat
    {
        msg::GnssID gnss_id;
        std::uint8_t sat_id;
        std::uint8_t signal_noise_ratio;
        std::uint8_t used :1;
    };

    static constexpr unsigned MaxSats = 128;

    Timestamps ts;

    float gdop;
    float pdop;
    float hdop;
    float vdop;
    float tdop;
    float ndop;
    float edop;

    unsigned num_sats;
    Sat sats[MaxSats];
};


struct ChecksumComputer
{
    std::uint8_t checksum_a = 0;
    std::uint8_t checksum_b = 0;

    void add(std::uint8_t byte);
    void add(const std::uint8_t* data, unsigned len);
};

class MessageReceiver
{
    // Suspicious invalid values
    static constexpr std::uint8_t InvalidClassValues[] = { 0x00, 0xFF };

    enum class State
    {
        SyncChar1,
        SyncChar2,
        Class,
        ID,
        Length1,
        Length2,
        Payload,
        ChecksumA,
        ChecksumB
    } state = State::SyncChar1;

    const unsigned payload_buffer_len;
    std::uint8_t* const payload_buffer;
    Timestamps sync_char_ts;
    std::uint16_t payload_len_expected = 0;
    std::uint16_t payload_len_received = 0;
    std::uint8_t cls = 0;
    std::uint8_t id = 0;
    std::uint8_t checksum_a = 0;
    std::uint8_t checksum_b = 0;

public:
    MessageReceiver(std::uint8_t* const arg_payload_buffer, unsigned arg_payload_buffer_length)
        : payload_buffer_len(arg_payload_buffer_length)
        , payload_buffer(arg_payload_buffer)
    { }

    bool nextByte(const Timestamps& ts, std::uint8_t byte);

    unsigned getReadLenHint() const;

    RxMessage getReceivedMessage() const;

    void reset();
};

class IPlatform
{
public:
    virtual ~IPlatform() { }
    virtual void portWrite(const std::uint8_t* data, unsigned len) = 0;
    virtual unsigned portRead(std::uint8_t* out_data, unsigned max_len, unsigned timeout_ms) = 0;
    virtual void portSetBaudRate(unsigned new_baudrate) = 0;
    virtual std::uint64_t getMonotonicUSec() const = 0;
    virtual std::uint64_t getRealUSec() const = 0;
};

class IOManager
{
    static constexpr unsigned PayloadBufferSize = 1024;
    static constexpr unsigned DefaultAckTimeoutMs = 200;
    static constexpr unsigned ValidBaudRates[] = { 9600, 115200, 57600, 38400, 19200, 4800 };
    static constexpr unsigned TargetBaudRate = 115200;

    struct LastAck
    {
        std::uint8_t cls = 0;
        std::uint8_t id = 0;
    };

    IPlatform& platform_;
    LastAck last_ack_;
    RxMessage last_received_msg_;
    std::uint8_t payload_buffer_[PayloadBufferSize];
    MessageReceiver rx_ = MessageReceiver(payload_buffer_, PayloadBufferSize);

    void handleReceivedMessage(const RxMessage& raw_msg);

public:
    IOManager(IPlatform& arg_platform)
        : platform_(arg_platform)
    { }

    bool configure(crdr_chibios::watchdog::Timer& wdt);

    void spin(unsigned timeout_ms);

    template <typename MessageStruct>
    MessageStruct* allocateMessage()
    {
        static_assert(sizeof(MessageStruct) <= PayloadBufferSize, "TX message is too long");
        std::fill_n(payload_buffer_, PayloadBufferSize, 0);
        auto ptr = reinterpret_cast<MessageStruct*>(payload_buffer_);
        *ptr = MessageStruct();
        return ptr;
    }

    void send(const Message& msg);
    bool sendAndWaitAck(const Message& msg, unsigned ack_timeout_ms = DefaultAckTimeoutMs);

    template <typename MessageStruct>
    const MessageStruct* poll(unsigned timeout_ms = DefaultAckTimeoutMs)
    {
        const auto deadline = platform_.getMonotonicUSec() + timeout_ms * 1000ULL;

        auto msg = Message();
        msg.cls = MessageStruct::Class;
        msg.id  = MessageStruct::ID;
        send(msg);

        last_received_msg_ = RxMessage();

        while (platform_.getMonotonicUSec() < deadline)
        {
            spin(0);
            if (auto msg = last_received_msg_.tryCastTo<MessageStruct>())
            {
                return msg;
            }
        }
        return nullptr;
    }

    std::uint64_t getMonotonicUSec() const { return platform_.getMonotonicUSec(); }

    std::function<void (const RxMessage&)> on_message;
};

/*
 * Driver core
 */
struct Config
{
    float fix_rate_hz = 10;
    float aux_rate_hz = 1;
};

class Driver
{
    IOManager io_;
    Config cfg_;
    std::uint64_t configured_at_mono_usec_ = 0;

    Fix fix_ = Fix();
    Aux aux_ = Aux();

    void handlePVT(const Timestamps& ts, const msg::NAV_PVT& pvt);
    void handleDOP(const Timestamps& ts, const msg::NAV_DOP& dop);
    void handleSAT(const Timestamps& ts, const msg::NAV_SAT& sat);

    void handleReceivedMessage(const RxMessage& msg);

    bool configureMessageRate(std::uint8_t cls, std::uint8_t id, std::uint8_t rate);
    bool configureGnss(crdr_chibios::watchdog::Timer& wdt);
    bool configureMessages();

public:
    Driver(IPlatform& arg_platform)
        : io_(arg_platform)
    { }

    bool configure(const Config& cfg, crdr_chibios::watchdog::Timer& wdt);

    void spin(unsigned timeout_ms);

    std::function<void (const Fix&)> on_fix;
    std::function<void (const Aux&)> on_aux;

    const Fix& getFix() const { return fix_; }
    const Aux& getAux() const { return aux_; }

    bool areRatesValid() const;
};

}
