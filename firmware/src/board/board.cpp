/*
 * Copyright (C) 2014-2015  Zubax Robotics  <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "board.hpp"
#include "stm32_flash_writer.hpp"
#include <cstring>
#include <ch.hpp>
#include <hal.h>
#include <zubax_chibios/os.hpp>
#include <unistd.h>

#if CORTEX_VTOR_INIT == 0
# error CORTEX_VTOR_INIT
#endif

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

/// Provided by linker
const extern std::uint8_t DeviceSignatureStorage[];

namespace board
{

static const I2CConfig I2CCfg1 =
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

    os::lowsyslog("Zubax GNSS %d.%d.%x\n", FW_VERSION_MAJOR, FW_VERSION_MINOR, GIT_HASH);
    os::watchdog::init();

    i2cStart(&I2CD1, &I2CCfg1);

    while (true)
    {
        const int res = os::config::init();
        if (res >= 0)
        {
            break;
        }
        os::lowsyslog("Config init failed %i\n", res);
        ::sleep(1);
    }
}

__attribute__((noreturn))
void die(int error)
{
    os::lowsyslog("Fatal error %i\n", error);
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
    SCB->ICSR = SCB_ICSR_PENDSVCLR;   // Clear all pending interrupts

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

void restart()
{
    NVIC_SystemReset();
}

void readUniqueID(UniqueID& out_bytes)
{
    std::memcpy(out_bytes.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);
}

bool tryReadDeviceSignature(DeviceSignature& out_sign)
{
    std::memcpy(out_sign.data(), &DeviceSignatureStorage[0], std::tuple_size<DeviceSignature>::value);

    bool valid = false;
    for (auto x : out_sign)
    {
        if (x != 0xFF && x != 0x00)          // All 0xFF/0x00 is not a valid signature, it's empty storage
        {
            valid = true;
            break;
        }
    }

    return valid;
}

bool tryWriteDeviceSignature(const DeviceSignature& sign)
{
    {
        DeviceSignature dummy;
        if (tryReadDeviceSignature(dummy))
        {
            return false;               // Already written
        }
    }

    // Before flash can be written, the source must be aligned.
    alignas(4) std::uint8_t aligned_buffer[std::tuple_size<DeviceSignature>::value];
    std::copy(std::begin(sign), std::end(sign), std::begin(aligned_buffer));

    stm32_flash_writer::Writer writer;

    return writer.write(&DeviceSignatureStorage[0], &aligned_buffer[0], sizeof(aligned_buffer));
}

HardwareVersion detectHardwareVersion()
{
    auto v = HardwareVersion();

    v.major = HW_VERSION;
    v.minor = 0;                // Some detection will be added in future versions

    return v;
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

    AFIO->MAPR = mapr | AFIO_MAPR_CAN_REMAP_REMAP2;

    /*
     * Enabling the CAN controllers, then configuring GPIO functions for CAN_TX.
     * Order matters, otherwise the CAN_TX pins will twitch, disturbing the CAN bus.
     * This is why we can't perform this initialization using ChibiOS GPIO configuration.
     *
     * NOTE: Check this - the problem may only appear when CAN pin remapping is used,
     *       because ChibiOS initializes AFIO after GPIO.
     */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    palSetPadMode(GPIOB, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

#if UAVCAN_STM32_NUM_IFACES > 1
    RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
    palSetPadMode(GPIOB, 13, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
#endif
}

}
