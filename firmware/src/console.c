/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <shell.h>
#include <crdr_chibios/config/cli.h>
#include <crdr_chibios/sys/sys.h>

static void printStatus(int err)
{
    if (err == 0)
        puts("OK");
    else
        printf("ERROR %d %s\n", err, strerror(abs(err)));
}

static void copySerialData(SerialDriver* src, SerialDriver* dst, int timeout_ms)
{
    uint8_t buffer[128];
    const unsigned sz = sdReadTimeout(src, buffer, sizeof(buffer), MS2ST(timeout_ms));
    if (sz > 0)
        sdWrite(dst, buffer, sz);
}

// --------------------------

static void cmd_cfg(BaseSequentialStream* bss, int argc, char *argv[])
{
    (void)bss;
    const int res = configExecuteCliCommand(argc, argv);
    if (res)
        printStatus(res);
}

static void cmd_ubxbridge(BaseSequentialStream* bss, int argc, char *argv[])
{
    (void)bss;
    (void)argc;
    (void)argv;
    while (1)
    {
        copySerialData(&UBLOX_SD, &STDOUT_SD, 1);
        copySerialData(&STDOUT_SD, &UBLOX_SD, 1);
    }
}

#define COMMAND(cmd)    {#cmd, cmd_##cmd},
static const ShellCommand _commands[] =
{
    COMMAND(cfg)
    COMMAND(ubxbridge)
    {NULL, NULL}
};

// --------------------------

static const ShellConfig _config = {(BaseSequentialStream*)&STDOUT_SD, _commands};

static WORKING_AREA(_wa_shell, 1024);

int consoleInit(void)
{
    if (palReadPad(GPIO_PORT_SERIAL_RX, GPIO_PIN_SERIAL_RX) == 0) {
        lowsyslog("Console: RX pin is low, console will not be inited\n");
        return 0; // That's OK
    }
    shellInit();
    return shellCreateStatic(&_config, _wa_shell, sizeof(_wa_shell), LOWPRIO) ? 0 : -1;
}
