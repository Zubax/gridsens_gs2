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

// --------------------------

static void cmd_cfg(BaseSequentialStream* bss, int argc, char *argv[])
{
    (void)bss;
    const int res = configExecuteCliCommand(argc, argv);
    if (res)
        printStatus(res);
}

#define COMMAND(cmd)    {#cmd, cmd_##cmd},
static const ShellCommand _commands[] =
{
    COMMAND(cfg)
    {NULL, NULL}
};

// --------------------------

static const ShellConfig _config = {(BaseSequentialStream*)&STDOUT_SD, _commands};

static WORKING_AREA(_wa_shell, 1024);

void consoleInit(void)
{
    if (palReadPad(GPIO_PORT_SERIAL_RX, GPIO_PIN_SERIAL_RX) == 0) {
        lowsyslog("Console: RX pin is low, console will not be inited\n");
        return;
    }
    shellInit();
    ASSERT_ALWAYS(shellCreateStatic(&_config, _wa_shell, sizeof(_wa_shell), LOWPRIO));
}
