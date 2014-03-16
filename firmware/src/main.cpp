/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cassert>
#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/config/config.hpp>
#include <crdr_chibios/watchdog/watchdog.hpp>


static crdr_chibios::config::Param<int> my_integer_param("my_int", 18, 0, 99);
static crdr_chibios::config::Param<double> my_double_param("my_double", -34, -99, 99);
static crdr_chibios::config::Param<bool> my_boolean_param("my_bool", false);

static crdr_chibios::watchdog::Timer wdt;

static void ledStatusSet(bool state)
{
    palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, !state);
}

static int init()
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

    crdr_chibios::watchdog::init();

    // Config
    const int config_res = crdr_chibios::config::init();
    if (config_res != 0)
        return config_res;

    // Console
    usleep(100000);
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

int main()
{
    const int init_res = init();
    if (init_res)
        die(init_res);

    // Just a test
    lowsyslog("Params: %i %lf %i\n", my_integer_param.get(), my_double_param.get(), int(my_boolean_param));

    wdt.startMSec(1000);

    bool led_state = false;
    while (1)
    {
        usleep(500000);
        ledStatusSet(led_state);
        led_state = !led_state;
        wdt.reset();
    }

    return 0;
}
