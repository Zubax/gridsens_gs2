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

static int init(void)
{
    halInit();
    chSysInit();
    sdStart(&STDOUT_SD, NULL);

    // ublox serial port
    const SerialConfig ubx_cfg =
    {
        9600,
        0,
        USART_CR2_STOP1_BITS | USART_CR2_LINEN,
        0
    };
    sdStart(&UBLOX_SD, &ubx_cfg);

    // Config
    const int config_res = configInit();
    if (config_res != 0)
        return config_res;

    // Console
    int consoleInit(void);
    const int console_res = consoleInit();
    if (console_res != 0)
        return console_res;

    return 0;
}

__attribute__((noreturn))
static void die(int status)
{
    lowsyslog("Now I am dead x_x %i\n", status);
    while (1) {
        ledStatusSet(false);
        sleep(1);
        ledStatusSet(true);
        sleep(1);
    }
}

int main(void)
{
    const int init_res = init();
    if (init_res)
        die(init_res);

    bool led_state = false;
    while (1)
    {
        usleep(500000);
        ledStatusSet(led_state);
        led_state = !led_state;
    }

    return 0;
}
