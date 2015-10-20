/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "cli.hpp"
#include "gnss.hpp"
#include "board/board.hpp"
#include <unistd.h>
#include <cstdio>
#include <zubax_chibios/sys/sys.h>
#include <zubax_chibios/sys/assert_always.h>
#include <zubax_chibios/config/cli.hpp>
#include <zubax_chibios/watchdog/watchdog.hpp>
#include <ch.hpp>
#include <hal.h>
#include <shell.h>

namespace cli
{

namespace
{

void cmd_cfg(BaseSequentialStream*, int argc, char* argv[])
{
    zubax_chibios::config::executeCliCommand(argc, argv);
}

void cmd_reset(BaseSequentialStream*, int, char**)
{
    lowsyslog("RESTART\n\n");
    ::usleep(10000);
    ::NVIC_SystemReset();
}

void cmd_gnssbridge(BaseSequentialStream*, int, char**)
{
    lowsyslog("\nRESET THE BOARD TO RESUME NORMAL OPERATION\n\n");
    gnss::stop();
    ::sleep(1);

    SerialDriver& gnss_port = gnss::getSerialPort();
    SerialDriver& cli_port = STDOUT_SD;

    static const auto copy_once = [](SerialDriver* src, SerialDriver* dst)
    {
        uint8_t buffer[128];
        const unsigned sz = sdReadTimeout(src, buffer, sizeof(buffer), TIME_IMMEDIATE);
        if (sz > 0)
        {
            sdWrite(dst, buffer, sz);
        }
    };

    while (true)
    {
        copy_once(&gnss_port, &cli_port);
        copy_once(&cli_port, &gnss_port);
    }
}

void cmd_bootloader(BaseSequentialStream*, int, char**)
{
    lowsyslog("\nENTERING THE BOOTLOADER\n\n");
    ::usleep(100000);

    // Suppress the watchdog - set the maximum possible interval
    zubax_chibios::watchdog::Timer().startMSec(1000000);

    board::enterBootloader();
}

const char* signatureToHex(const board::DeviceSignature& sign,
                           char out_str[std::tuple_size<board::DeviceSignature>::value * 2 + 1])
{
    static const auto nibble2hex = [](std::uint8_t x)
    {
        const char n = char((x & 0xF) + '0');
        return (n > '9') ? char(n + 'A' - '9' - 1) : n;
    };

    unsigned pos = 0;
    for (auto x : sign)
    {
        out_str[pos++] = nibble2hex(x >> 4);
        out_str[pos++] = nibble2hex(x);
    }

    out_str[std::tuple_size<board::DeviceSignature>::value * 2] = '\0';

    return &out_str[0];
}

void cmd_signature(BaseSequentialStream*, int argc, char** argv)
{
    if (argc == 0)
    {
        board::DeviceSignature sign;
        if (board::tryReadDeviceSignature(sign))
        {
            char buf[257];
            std::puts(signatureToHex(sign, buf));
        }
        else
        {
            std::puts("Error: Could not read signature");
        }
    }
    else
    {
        const char* const sign = argv[0];
        (void)sign;
    }
}

const ::ShellCommand HandlerTable[] =
{
    {"cfg",        &cmd_cfg},
    {"reset",      &cmd_reset},
    {"gnssbridge", &cmd_gnssbridge},
    {"bootloader", &cmd_bootloader},
    {"signature",  &cmd_signature},
    {nullptr, nullptr}
};

#if defined(RELEASE_BUILD) && RELEASE_BUILD
bool readRxPin()
{
    return palReadPad(GPIO_PORT_SERIAL_RX, GPIO_PIN_SERIAL_RX);
}
#endif

}

void init()
{
#if defined(RELEASE_BUILD) && RELEASE_BUILD
    if (!readRxPin())
    {
        ::usleep(500000);   // Some USB-serial adapters are weird and need some time to initialize TX line
        if (!readRxPin())
        {
            lowsyslog("Console: RX pin is low, console will not be inited\n");
            return;
        }
    }
#endif

    ::shellInit();

    static WORKING_AREA(_wa_shell, 1024);

    static const ShellConfig config
    {
        (BaseSequentialStream*)&STDOUT_SD,
        HandlerTable
    };
    const Thread* const thd = ::shellCreateStatic(&config, _wa_shell, sizeof(_wa_shell), LOWPRIO);
    ASSERT_ALWAYS(thd != nullptr);
}

}
