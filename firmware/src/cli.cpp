/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include "cli.hpp"
#include "gnss.hpp"
#include <unistd.h>
#include <crdr_chibios/sys/sys.h>
#include <crdr_chibios/sys/assert_always.h>
#include <crdr_chibios/config/cli.hpp>
#include <crdr_chibios/watchdog/watchdog.hpp>
#include <ch.hpp>
#include <hal.h>
#include <shell.h>

namespace cli
{

namespace
{

void cmd_cfg(BaseSequentialStream*, int argc, char* argv[])
{
    crdr_chibios::config::executeCliCommand(argc, argv);
}

void cmd_reset(BaseSequentialStream*, int, char**)
{
    lowsyslog("RESTART\n\n");
    ::usleep(10000);
    ::NVIC_SystemReset();
}

void cmd_gnssbridge(BaseSequentialStream*, int, char**)
{
    lowsyslog("\nRESET THE BOARD TO RESUME NORMAL OPERATION\n\n");
    gnss::stop();
    ::sleep(1);

    SerialDriver& gnss_port = gnss::getSerialPort();
    SerialDriver& cli_port = STDOUT_SD;

    static const auto copy_once = [](SerialDriver* src, SerialDriver* dst)
    {
        uint8_t buffer[128];
        const unsigned sz = sdReadTimeout(src, buffer, sizeof(buffer), TIME_IMMEDIATE);
        if (sz > 0)
        {
            sdWrite(dst, buffer, sz);
        }
    };

    while (true)
    {
        copy_once(&gnss_port, &cli_port);
        copy_once(&cli_port, &gnss_port);
    }
}

void cmd_bootloader(BaseSequentialStream*, int, char**)
{
    static const unsigned long BootloaderAddress = 0x1FFFB000UL;
    const unsigned long BootloaderEntryPoint = *reinterpret_cast<unsigned long*>(BootloaderAddress + 4);

    lowsyslog("\nENTERING THE BOOTLOADER AT 0x%08x\n\n", unsigned(BootloaderEntryPoint));

    // Suppress the watchdog - set the maximum possible interval
    crdr_chibios::watchdog::Timer().startMSec(1000000);

    ::usleep(100000);

    // Disable all interrupts
    __set_PRIMASK(1);
    for (int i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = NVIC->IABR[i];
    }
    SCB_ICSR = ICSR_PENDSVCLR;   // Clear all pending interrupts

    // Reset RCC
    RCC->CR |= 0x00000001UL;
    RCC->CFGR &= 0xF8FF0000UL;
    RCC->CR &= 0xFEF6FFFFUL;
    RCC->CR &= 0xFFFBFFFFUL;
    RCC->CFGR &= 0xFF80FFFFUL;
    RCC->CR &= 0xEBFFFFFFUL;
    RCC->CIR = 0x00FF0000UL;
    RCC->CFGR2 = 0x00000000UL;

    // Reset the sys tick timer
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Clock from HSI
    {
        unsigned long tmpreg = RCC->CFGR;
        tmpreg &= 0xFFFFFFFCUL;
        RCC->CFGR = tmpreg;
    }

    // Reset USART
    RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    RCC->APB2RSTR |= ~RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &=  RCC_APB2RSTR_USART1RST;

    // Call the bootloader
    __set_MSP(0x20001000UL);
    reinterpret_cast<void (*)()>(BootloaderEntryPoint)();

    while (true) { }
}

const ::ShellCommand HandlerTable[] =
{
    {"cfg",        &cmd_cfg},
    {"reset",      &cmd_reset},
    {"gnssbridge", &cmd_gnssbridge},
    {"bootloader", &cmd_bootloader},
    {nullptr, nullptr}
};

#if RELEASE
bool readRxPin()
{
    return palReadPad(GPIO_PORT_SERIAL_RX, GPIO_PIN_SERIAL_RX);
}
#endif

}

void init()
{
#if RELEASE
    if (!readRxPin())
    {
        ::usleep(500000);   // Some USB-serial adapters are weird and need some time to initialize TX line
        if (!readRxPin())
        {
            lowsyslog("Console: RX pin is low, console will not be inited\n");
            return;
        }
    }
#endif

    ::shellInit();

    static WORKING_AREA(_wa_shell, 1024);

    static const ShellConfig config
    {
        (BaseSequentialStream*)&STDOUT_SD,
        HandlerTable
    };
    const Thread* const thd = ::shellCreateStatic(&config, _wa_shell, sizeof(_wa_shell), LOWPRIO);
    ASSERT_ALWAYS(thd != nullptr);
}

}
