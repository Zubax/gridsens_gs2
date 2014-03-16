/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <unistd.h>
#include <ch.hpp>
#include <hal.h>
#include <shell.h>
#include <crdr_chibios/config/cli.hpp>
#include <crdr_chibios/sys/sys.h>

static void printIfError(int err)
{
    if (err)
        std::printf("ERROR %d %s\n", err, std::strerror(std::abs(err)));
}

struct Handlers
{
    static void cfg(BaseSequentialStream*, int argc, char *argv[])
    {
        printIfError(crdr_chibios::config::executeCliCommand(argc, argv));
    }

    static void ubxbridge(BaseSequentialStream*, int, char *[])
    {
        static auto copy_once = [](SerialDriver* src, SerialDriver* dst)
        {
            uint8_t buffer[128];
            const unsigned sz = sdReadTimeout(src, buffer, sizeof(buffer), MS2ST(1));
            if (sz > 0)
                sdWrite(dst, buffer, sz);
        };
        while (1)
        {
            copy_once(&UBLOX_SD, &STDOUT_SD);
            copy_once(&STDOUT_SD, &UBLOX_SD);
        }
    }

    static const ShellCommand Table[];
};

#define COMMAND(cmd)    {#cmd, &Handlers::cmd},
const ShellCommand Handlers::Table[] =
{
    COMMAND(cfg)
    COMMAND(ubxbridge)
    {NULL, NULL}
};

#if RELEASE
static bool readRxPin(void)
{
    return palReadPad(GPIO_PORT_SERIAL_RX, GPIO_PIN_SERIAL_RX);
}
#endif

int consoleInit(void)
{
#if RELEASE
    if (!readRxPin())
    {
        usleep(500000);   // Some USB-serial adapters are weird and need some time to initialize TX line
        if (!readRxPin())
        {
            lowsyslog("Console: RX pin is low, console will not be inited\n");
            return 0;     // That's OK
        }
    }
#endif

    shellInit();

    static WORKING_AREA(_wa_shell, 1024);

    static const ShellConfig config
    {
        (BaseSequentialStream*)&STDOUT_SD,
        Handlers::Table
    };
    return shellCreateStatic(&config, _wa_shell, sizeof(_wa_shell), LOWPRIO) ? 0 : -1;
}
