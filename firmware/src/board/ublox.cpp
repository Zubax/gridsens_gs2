/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "ublox.hpp"
#include <ctime>
#include <crdr_chibios/sys/sys.h>

namespace ublox
{
/*
 * MessageReceiver::ChecksumComputer
 */
void ChecksumComputer::add(std::uint8_t byte)
{
    checksum_a += byte;
    checksum_b += checksum_a;
}

void ChecksumComputer::add(const std::uint8_t* data, unsigned len)
{
    while (len --> 0)
    {
        add(*data++);
    }
}

/*
 * MessageReceiver
 */
constexpr std::uint8_t MessageReceiver::InvalidClassValues[];

bool MessageReceiver::nextByte(const Timestamps& ts, std::uint8_t byte)
{
    bool ret_value = false;

    //lowsyslog("%02x ", int(byte));

    switch (state)
    {
    case State::SyncChar1:
    {
        if (byte == 0xB5)
        {
            state = State::SyncChar2;
            sync_char_ts = ts;
        }
        break;
    }
    case State::SyncChar2:
    {
        state = (byte == 0x62) ? State::Class : State::SyncChar1;
        break;
    }
    case State::Class:
    {
        for (auto val : InvalidClassValues)
        {
            if (byte == val)
            {
                reset();
                break;
            }
        }
        cls = byte;
        state = State::ID;
        break;
    }
    case State::ID:
    {
        id = byte;
        state = State::Length1;
        break;
    }
    case State::Length1:
    {
        payload_len_expected = byte;
        state = State::Length2;
        break;
    }
    case State::Length2:
    {
        payload_len_expected |= static_cast<std::uint16_t>(byte) << 8;
        payload_len_received = 0;
        if (payload_len_expected <= payload_buffer_len)
        {
            state = State::Payload;
        }
        else
        {
            // The message is too long, likely invalid
            lowsyslog("ublox: Msg too long %d\n", int(payload_len_expected));
            reset();
        }
        break;
    }
    case State::Payload:
    {
        // Will not work wit zero-length payload
        payload_buffer[payload_len_received] = byte;
        payload_len_received++;
        if (payload_len_received >= payload_len_expected)
        {
            state = State::ChecksumA;
        }
        break;
    }
    case State::ChecksumA:
    {
        checksum_a = byte;
        state = State::ChecksumB;
        break;
    }
    case State::ChecksumB:
    {
        checksum_b = byte;
        state = State::SyncChar1;   // This message is now compelte, restart

        ChecksumComputer ckc;
        ckc.add(cls);
        ckc.add(id);
        ckc.add(payload_len_expected & 0xFF);
        ckc.add((payload_len_expected >> 8) & 0xFF);
        ckc.add(payload_buffer, payload_len_received);

        ret_value = (ckc.checksum_a == checksum_a) && (ckc.checksum_b == checksum_b);
        if (!ret_value)
        {
            lowsyslog("ublox: Invalid checksum\n");
        }
        break;
    }
    default:
    {
        assert(0);
        reset();
    }
    }

    return ret_value;
}

unsigned MessageReceiver::getReadLenHint() const
{
    unsigned retval = 1;
    if ((state == State::Payload) && (payload_len_expected > payload_len_received))
    {
        retval = payload_len_expected - payload_len_received;
    }
    assert(retval > 0);
    return retval;
}

RxMessage MessageReceiver::getReceivedMessage() const
{
    RxMessage msg{};
    msg.mono_usec = sync_char_ts.mono_usec;
    msg.real_usec = sync_char_ts.real_usec;
    msg.cls = cls;
    msg.id = id;
    msg.len = payload_len_received;
    msg.payload = payload_buffer;
    assert(msg.len > 0);
    assert(msg.mono_usec > 0);
    return msg;
}

void MessageReceiver::reset()
{
    state = State::SyncChar1;
}

/*
 * IOManager
 */
constexpr unsigned IOManager::ValidBaudRates[];

void IOManager::handleReceivedMessage(const RxMessage& raw_msg)
{
    //lowsyslog("ublox: Rx 0x%02x 0x%02x %d\n", int(raw_msg.cls), int(raw_msg.id), int(raw_msg.len));

    if (auto msg = raw_msg.tryCastTo<msg::ACK_ACK>())
    {
        last_ack_.cls = msg->clsID;
        last_ack_.id  = msg->msgID;
        return;
    }
    else if (auto msg = raw_msg.tryCastTo<msg::ACK_NAK>())
    {
        lowsyslog("ublox: NAK 0x%02x 0x%02x\n", int(msg->clsID), int(msg->msgID));
        return;
    }
    else
    {
        if (on_message)
        {
            on_message(raw_msg);
        }
    }
}

bool IOManager::configure(crdr_chibios::watchdog::Timer& wdt)
{
    auto prt_uart = msg::CFG_PRT_UART();
    prt_uart.portID = msg::CFG_PRT_UART::PortID::UART1;
    prt_uart.mode = msg::CFG_PRT_UART::Mode8N1;
    prt_uart.baudRate = TargetBaudRate;
    prt_uart.inProtoMask  = msg::CFG_PRT_UART::InProtoMask::inUbx | msg::CFG_PRT_UART::InProtoMask::inRtcm;
    prt_uart.outProtoMask = msg::CFG_PRT_UART::OutProtoMask::outUbx;

    bool baudrate_detected = false;
    for (unsigned baudrate : ValidBaudRates)
    {
        wdt.reset();
        const auto last_msg_ts = last_received_msg_.mono_usec;

        lowsyslog("ublox: Trying baudrate %u...\n", baudrate);
        rx_.reset();
        platform_.portSetBaudRate(baudrate);

        // Set the same baudrate, wait for any valid message in response
        prt_uart.baudRate = baudrate;
        if (sendAndWaitAck(Message::make(prt_uart)) || (last_received_msg_.mono_usec > last_msg_ts))
        {
            lowsyslog("ublox: Baudrate match %u\n", baudrate);
            baudrate_detected = true;
            break;
        }
    }

    if (!baudrate_detected)
    {
        lowsyslog("Failed to detect baudrate\n");
        return false;
    }

    wdt.reset();
    prt_uart.baudRate = TargetBaudRate;
    (void)sendAndWaitAck(Message::make(prt_uart));
    platform_.portSetBaudRate(TargetBaudRate);
    rx_.reset();

    return true;
}

void IOManager::spin(unsigned timeout_ms)
{
    const auto deadline = platform_.getMonotonicUSec() + timeout_ms * 1000ULL;
    do
    {
        /*
         * Read from the port
         */
        constexpr unsigned ReadTimeoutMs = 1;
        constexpr unsigned BufferSize = 16;
        std::uint8_t buffer[BufferSize];
        Timestamps read_ts;
        unsigned bytes_read = std::min(rx_.getReadLenHint(), BufferSize);
        bytes_read = platform_.portRead(buffer, bytes_read, ReadTimeoutMs);
        read_ts.real_usec = platform_.getRealUSec();
        read_ts.mono_usec = platform_.getMonotonicUSec();

        /*
         * Pass the data to the receiver state machine byte-by-byte
         */
        const std::uint8_t* byte_ptr = buffer;
        while (bytes_read --> 0)
        {
            if (rx_.nextByte(read_ts, *byte_ptr++))
            {
                last_received_msg_ = rx_.getReceivedMessage();
                handleReceivedMessage(last_received_msg_);
            }
        }
    }
    while (platform_.getMonotonicUSec() < deadline);
}

void IOManager::send(const Message& msg)
{
    ChecksumComputer ckc;
    ckc.add(msg.cls);
    ckc.add(msg.id);
    ckc.add(msg.len & 0xFF);
    ckc.add((msg.len >> 8) & 0xFF);
    ckc.add(msg.payload, msg.len);

    static const std::uint8_t Header[] = { 0xB5, 0x62 };

    const std::uint8_t class_id[] = { msg.cls, msg.id };
    const std::uint8_t length[] = { std::uint8_t(msg.len & 0xFF), std::uint8_t((msg.len >> 8) & 0xFF) };
    const std::uint8_t checksum[] = { ckc.checksum_a, ckc.checksum_b };

    platform_.portWrite(Header, 2);
    platform_.portWrite(class_id, 2);
    platform_.portWrite(length, 2);
    if (msg.len > 0)
    {
        assert(msg.payload != nullptr);
        platform_.portWrite(msg.payload, msg.len);
    }
    platform_.portWrite(checksum, 2);
}

bool IOManager::sendAndWaitAck(const Message& msg, unsigned ack_timeout_ms)
{
    const auto deadline = platform_.getMonotonicUSec() + ack_timeout_ms * 1000ULL;

    send(msg);

    last_ack_ = LastAck();

    while (platform_.getMonotonicUSec() < deadline)
    {
        spin(0);
        if ((last_ack_.cls == msg.cls) && (last_ack_.id == msg.id))
        {
            return true;
        }
    }
    return false;
}

/*
 * Driver
 */
void Driver::handlePVT(const Timestamps& ts, const msg::NAV_PVT& pvt)
{
    fix_ = Fix();
    fix_.ts = ts;

    // Position
    fix_.lat = pvt.lat / 1e7;
    fix_.lon = pvt.lon / 1e7;
    fix_.alt = pvt.height / 1e3F;

    // Velocity
    fix_.ned_velocity[0] = pvt.velN / 1e3F;
    fix_.ned_velocity[1] = pvt.velE / 1e3F;
    fix_.ned_velocity[2] = pvt.velD / 1e3F;

    // Uncertainties
    static const auto variance_1e3 = [](const std::uint32_t x) { return (x / 1e3F) * (x / 1e3F); };

    fix_.position_covariance[0] = variance_1e3(pvt.hAcc);
    fix_.position_covariance[4] = variance_1e3(pvt.hAcc);
    fix_.position_covariance[8] = variance_1e3(pvt.vAcc);

    fix_.velocity_covariance[0] = variance_1e3(pvt.sAcc);
    fix_.velocity_covariance[4] = variance_1e3(pvt.sAcc);
    fix_.velocity_covariance[8] = variance_1e3(pvt.sAcc);

    fix_.pdop = pvt.pDOP * 0.01F;

    // Fix mode
    switch (pvt.fixType)
    {
    case msg::NAV_PVT::FixType::Fix2D:
        fix_.mode = Fix::Mode::Fix2D;
        break;

    case msg::NAV_PVT::FixType::Fix3D:
    case msg::NAV_PVT::FixType::GnssAndDeadReckoning:
        fix_.mode = Fix::Mode::Fix3D;
        break;

    case msg::NAV_PVT::FixType::TimeOnly:
        fix_.mode = Fix::Mode::Time;
        break;

    case msg::NAV_PVT::FixType::DeadReckoning:
    case msg::NAV_PVT::FixType::NoFix:
    default:
        fix_.mode = Fix::Mode::None;
        break;
    }

    // Misc
    fix_.sats_used = pvt.numSV;
    if (pvt.flags & msg::NAV_PVT::FlagsMask::diffSoln)
    {
        fix_.flags |= Fix::Flags::DifferentialSolution;
    }

    // UTC validness flag
    static const auto UtcValidFlags = msg::NAV_PVT::ValidMask::validDate | msg::NAV_PVT::ValidMask::validTime;
    fix_.utc_valid = ((pvt.valid & UtcValidFlags) == UtcValidFlags) && (pvt.year > 1900) && (pvt.month > 0);

    // UTC timestamp computation
    if (fix_.utc_valid)
    {
        auto tm = std::tm();
        tm.tm_year = pvt.year - 1900;
        tm.tm_mon  = pvt.month - 1;
        tm.tm_mday = pvt.day;
        tm.tm_hour = pvt.hour;
        tm.tm_min  = pvt.min;
        tm.tm_sec  = pvt.sec;
        tm.tm_isdst = 0;

        std::int64_t signed_usec = static_cast<std::int64_t>(std::mktime(&tm)) * 1000000LL;
        signed_usec += pvt.nano / 1000;      // nano may be negative, hence the timestamp is signed
        fix_.utc_usec = signed_usec;
    }

    // Report update
    if (on_fix)
    {
        on_fix(fix_);
    }
}

void Driver::handleDOP(const Timestamps& ts, const msg::NAV_DOP& dop)
{
    aux_.ts = ts;
    aux_.gdop = dop.gDOP * 0.01F;
    aux_.pdop = dop.pDOP * 0.01F;
    aux_.hdop = dop.hDOP * 0.01F;
    aux_.vdop = dop.vDOP * 0.01F;
    aux_.tdop = dop.tDOP * 0.01F;
    aux_.ndop = dop.nDOP * 0.01F;
    aux_.edop = dop.eDOP * 0.01F;

    if (on_aux)
    {
        on_aux(aux_);
    }
}

void Driver::handleSAT(const Timestamps& ts, const msg::NAV_SAT& sat)
{
    (void)ts;
    if (sat.version != msg::NAV_SAT::MsgVersion)
    {
        lowsyslog("ublox: NAV-SAT of unsupported version %i\n", int(sat.version));
        return;
    }

    aux_.num_sats = sat.numSvs;

    for (unsigned i = 0; i < std::min(static_cast<unsigned>(sat.numSvs), Aux::MaxSats); i++)
    {
        const auto src = sat.svs[i];
        auto& dst = aux_.sats[i];

        dst.gnss_id = src.gnssId;
        dst.sat_id = src.svId;
        dst.signal_noise_ratio = src.cno;
        dst.used = (src.flags & msg::NAV_SAT::Sv::FlagsMask::svUsed) != 0;
    }
}

void Driver::handleReceivedMessage(const RxMessage& raw_msg)
{
    if (auto msg = raw_msg.tryCastTo<msg::NAV_PVT>())
    {
        handlePVT(raw_msg, *msg);
    }
    else if (auto msg = raw_msg.tryCastTo<msg::NAV_DOP>())
    {
        handleDOP(raw_msg, *msg);
    }
    else if (auto msg = raw_msg.tryCastTo<msg::NAV_SAT>())
    {
        handleSAT(raw_msg, *msg);
    }
    else
    {
        lowsyslog("ublox: Unknown message: class=0x%02x id=0x%02x payload_len=%d\n",
                  int(raw_msg.cls), int(raw_msg.id), int(raw_msg.len));
    }
}

bool Driver::configureMessageRate(std::uint8_t cls, std::uint8_t id, std::uint8_t rate)
{
    msg::CFG_MSG msg;
    msg.msgClass = cls;
    msg.msgID    = id;
    msg.rate     = rate;
    return io_.sendAndWaitAck(Message::make(msg));
}

bool Driver::configureGnss(crdr_chibios::watchdog::Timer& wdt)
{
    // Nav rate
    wdt.reset();
    {
        auto rate = io_.allocateMessage<msg::CFG_RATE>();
        rate->measRate = 1000 / cfg_.fix_rate_hz;
        rate->navRate = 1;                           // Required by the spec
        rate->timeRef = msg::CFG_RATE::TimeRef::UTC;
        if (!io_.sendAndWaitAck(Message::make(*rate)))
        {
            lowsyslog("ublox: CFG-RATE failed\n");
            return false;
        }
    }

    // GNSS configuration
    wdt.reset();
    msg::MON_GNSS mon_gnss;
    {
        auto mon_gnss_ptr = io_.poll<msg::MON_GNSS>();
        if (mon_gnss_ptr == nullptr)
        {
            lowsyslog("ublox: Failed to poll MON-GNSS\n");
            return false;
        }
        mon_gnss = *mon_gnss_ptr;
    }
    lowsyslog("ublox: MON-GNSS supported=%u default=%u enabled=%u simultaneous=%u\n",
              unsigned(mon_gnss.supported), unsigned(mon_gnss.default_),
              unsigned(mon_gnss.enabled), unsigned(mon_gnss.simultaneous));
    /*
     * Here we could set-up the new configuration, but we won't because
     * receivers are properly configured by default.
     */

    // NAV5
    wdt.reset();
    {
        auto nav5 = io_.allocateMessage<msg::CFG_NAV5>();
        nav5->mask = msg::CFG_NAV5::Mask::dyn;
        nav5->dynModel = msg::CFG_NAV5::DynModel::Airborne_2g;
        if (!io_.sendAndWaitAck(Message::make(*nav5)))
        {
            lowsyslog("ublox: CFG-NAV5 failed\n");
            return false;
        }
    }

    return true;
}

bool Driver::configureMessages()
{
    if (!configureMessageRate(msg::NAV_PVT::Class, msg::NAV_PVT::ID, 1))  // Position-Velocity-Time solution
    {
        return false;
    }
    {
        unsigned dop_rate = (cfg_.fix_rate_hz / cfg_.aux_rate_hz + 0.5F);
        dop_rate = std::min(0xFFU, dop_rate);
        dop_rate = std::max(1U, dop_rate);
        if (!configureMessageRate(msg::NAV_DOP::Class, msg::NAV_DOP::ID, dop_rate))  // DOP values
        {
            return false;
        }
    }
    {
        unsigned sat_rate = (cfg_.fix_rate_hz / std::min(cfg_.aux_rate_hz, 1.0F) + 0.5F);
        sat_rate = std::min(0xFFU, sat_rate);
        sat_rate = std::max(1U, sat_rate);
        if (!configureMessageRate(msg::NAV_SAT::Class, msg::NAV_SAT::ID, sat_rate)) // Sat info
        {
            return false;
        }
    }
    return true;
}

bool Driver::configure(const Config& cfg, crdr_chibios::watchdog::Timer& wdt)
{
    wdt.reset();
    if (!io_.on_message)
    {
        io_.on_message = std::bind(&Driver::handleReceivedMessage, this, std::placeholders::_1);
    }

    cfg_ = cfg;
    cfg_.fix_rate_hz = std::max(cfg_.fix_rate_hz, 0.5F);
    cfg_.aux_rate_hz = std::max(cfg_.aux_rate_hz, 0.1F);
    cfg_.aux_rate_hz = std::min(cfg_.aux_rate_hz, cfg_.fix_rate_hz);

    wdt.reset();
    if (!io_.configure(wdt))
    {
        return false;
    }

    wdt.reset();
    if (!configureGnss(wdt))
    {
        return false;
    }

    wdt.reset();
    if (!configureMessages())
    {
        return false;
    }

    configured_at_mono_usec_ = io_.getMonotonicUSec();

    wdt.reset();
    return true;
}

void Driver::spin(unsigned timeout_ms)
{
    io_.spin(timeout_ms);
}

bool Driver::areRatesValid() const
{
    constexpr unsigned IntervalMultiplier = 3;
    const auto ts_mono_usec = io_.getMonotonicUSec();

    static const auto validate = [=](const Timestamps& ts, float expected_rate)
    {
        const auto max_interval_usec = static_cast<std::uint64_t>(1e6 / expected_rate + 0.5F) * IntervalMultiplier;
        if (ts_mono_usec > max_interval_usec)
        {
            const auto mono_usec = std::max(configured_at_mono_usec_, ts.mono_usec);
            if (mono_usec < (ts_mono_usec - max_interval_usec))
            {
                return false;
            }
        }
        return true;
    };

    return validate(getFix().ts, cfg_.fix_rate_hz) && validate(getAux().ts, cfg_.aux_rate_hz);
}

}
