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

static void ledStatusSet(bool state)
{
    palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, !state);
}

int main(void)
{
    halInit();
    chSysInit();
    sdStart(&STDOUT_SD, NULL);

    bool led_state = false;
    while (1)
    {
        usleep(500000);
        ledStatusSet(led_state);
        lowsyslog("Hi!\n");
        led_state = !led_state;
    }

    return 0;
}
