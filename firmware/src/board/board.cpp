/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "board.hpp"
#include <cstring>
#include <ch.hpp>
#include <hal.h>
#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/watchdog/watchdog.hpp>
#include <crdr_chibios/config/config.hpp>
#include <unistd.h>

/**
 * GPIO config for ChibiOS PAL driver
 */
const PALConfig pal_default_config =
{
    { VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH },
    { VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH },
    { VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH },
    { VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH },
    { VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH }
};


namespace board
{

static const I2CConfig I2CCfg2 =
{
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

void init()
{
    halInit();
    chSysInit();
    sdStart(&STDOUT_SD, NULL);
    i2cStart(&I2CD2, &I2CCfg2);

    crdr_chibios::watchdog::init();

    while (true)
    {
        const int res = crdr_chibios::config::init();
        if (res >= 0)
        {
            break;
        }
        lowsyslog("Config init failed %i\n", res);
        ::sleep(1);
    }
}

__attribute__((noreturn))
void die(int error)
{
    lowsyslog("Fatal error %i\n", error);
    while (1)
    {
        setStatusLed(false);
        ::usleep(25000);
        setStatusLed(true);
        ::usleep(25000);
    }
}

void setCANLed(unsigned iface_index, bool state)
{
    switch (iface_index)
    {
    case 0:
    {
        palWritePad(GPIO_PORT_LED_CAN1, GPIO_PIN_LED_CAN1, state);
        break;
    }
    case 1:
    {
        palWritePad(GPIO_PORT_LED_CAN2, GPIO_PIN_LED_CAN2, state);
        break;
    }
    default:
    {
        break;
    }
    }
}

void setStatusLed(bool state)
{
    palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, state);
}

void readUniqueID(std::uint8_t bytes[UniqueIDSize])
{
    std::memcpy(bytes, reinterpret_cast<const void*>(0x1FFFF7E8), UniqueIDSize);
}

}

/*
 * Early init from ChibiOS
 */
extern "C"
{

void __early_init(void)
{
    stm32_clock_init();
}

void boardInit(void)
{
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~AFIO_MAPR_SWJ_CFG; // these bits are write-only

    // Enable SWJ only, JTAG is not needed at all:
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    AFIO->MAPR = mapr;
}

}
