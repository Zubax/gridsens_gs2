/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "board.hpp"
#include <cstring>
#include <ch.hpp>
#include <hal.h>
#include <zubax_chibios/sys/sys.h>
#include <zubax_chibios/watchdog/watchdog.hpp>
#include <zubax_chibios/config/config.hpp>
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

    zubax_chibios::watchdog::init();

    while (true)
    {
        const int res = zubax_chibios::config::init();
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

void enterBootloader()
{
    static const unsigned long BootloaderAddress = 0x1FFFB000UL;
    const unsigned long BootloaderEntryPoint = *reinterpret_cast<unsigned long*>(BootloaderAddress + 4);

    // Disable all interrupts
    __set_PRIMASK(1);
    for (int i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = NVIC->IABR[i];
    }
    SCB_ICSR = ICSR_PENDSVCLR;   // Clear all pending interrupts

    // Reset RCC
    RCC->CR   |= 0x00000001UL;
    RCC->CFGR &= 0xF8FF0000UL;
    RCC->CR   &= 0xFEF6FFFFUL;
    RCC->CR   &= 0xFFFBFFFFUL;
    RCC->CFGR &= 0xFF80FFFFUL;
    RCC->CR   &= 0xEBFFFFFFUL;
    RCC->CIR   = 0x00FF0000UL;
    RCC->CFGR2 = 0x00000000UL;

    // Reset the sys tick timer
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Clock from HSI
    RCC->CFGR = RCC->CFGR & 0xFFFFFFFCUL;

    // Reset USART
    RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    RCC->APB2RSTR |= ~RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &=  RCC_APB2RSTR_USART1RST;

    // Call the bootloader
    __set_MSP(0x20001000UL);
    reinterpret_cast<void (*)()>(BootloaderEntryPoint)();

    // Ford, you're turning into a penguin. Stop it.
    while (true) { }
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
