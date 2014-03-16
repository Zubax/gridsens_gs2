/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <ch.h>
#include <hal.h>
#include <assert.h>
#include <unistd.h>
#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/config/config.h>

CONFIG_PARAM_FLOAT("my_answer", 42, 0, 99)
CONFIG_PARAM_INT("my_int", 42, 0, 99)
CONFIG_PARAM_BOOL("my_bool", false)

static void ledStatusSet(bool state)
{
    palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, !state);
}

void consoleInit(void);

int main(void)
{
    halInit();
    chSysInit();
    sdStart(&STDOUT_SD, NULL);

    const int config_res = configInit();
    if (config_res != 0)
    {
        lowsyslog("Config init failed [%i]\n", config_res);
    }
    consoleInit();

    bool led_state = false;
    while (1)
    {
        usleep(500000);
        ledStatusSet(led_state);
        led_state = !led_state;
    }

    return 0;
}
