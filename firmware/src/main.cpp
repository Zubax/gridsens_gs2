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

int consoleInit();

namespace app
{

int uavcanInit();

namespace
{

void setStatusLed(bool state)
{
    palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, !state);
}

__attribute__((noreturn))
void die(int status)
{
    lowsyslog("Now I am dead x_x %i\n", status);
    while (1)
    {
        setStatusLed(false);
        sleep(1);
        setStatusLed(true);
        sleep(1);
    }
}

int init()
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
    {
        return config_res;
    }

    // UAVCAN stack
    const int uavcan_res = uavcanInit();
    if (uavcan_res != 0)
    {
        return uavcan_res;
    }

    // Console
    usleep(100000);
    const int console_res = consoleInit();
    if (console_res != 0)
    {
        return console_res;
    }

    return 0;
}

}
}

int main()
{
    const int init_res = app::init();
    if (init_res)
    {
        app::die(init_res);
    }

    bool led_state = false;
    while (1)
    {
        usleep(500000);
        app::setStatusLed(led_state);
        led_state = !led_state;
    }

    return 0;
}
