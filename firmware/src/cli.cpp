/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "cli.hpp"
#include "gnss.hpp"
#include "board/board.hpp"
#include <unistd.h>
#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/sys/assert_always.h>
#include <crdr_chibios/config/cli.hpp>
#include <crdr_chibios/watchdog/watchdog.hpp>
#include <ch.hpp>
#include <hal.h>
#include <shell.h>

namespace cli
{

namespace
{

void cmd_cfg(BaseSequentialStream*, int argc, char* argv[])
{
    crdr_chibios::config::executeCliCommand(argc, argv);
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
    crdr_chibios::watchdog::Timer().startMSec(1000000);

    board::enterBootloader();
}

const ::ShellCommand HandlerTable[] =
{
    {"cfg",        &cmd_cfg},
    {"reset",      &cmd_reset},
    {"gnssbridge", &cmd_gnssbridge},
    {"bootloader", &cmd_bootloader},
    {nullptr, nullptr}
};

#if RELEASE
bool readRxPin()
{
    return palReadPad(GPIO_PORT_SERIAL_RX, GPIO_PIN_SERIAL_RX);
}
#endif

}

void init()
{
#if RELEASE
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
