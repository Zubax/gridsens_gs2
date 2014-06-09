/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "cli.hpp"
#include <unistd.h>
#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/sys/assert_always.h>
#include <crdr_chibios/config/cli.hpp>
#include <ch.hpp>
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

const ::ShellCommand HandlerTable[] =
{
    "cfg",   &cmd_cfg,
    "reset", &cmd_reset,
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
