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

#pragma once

#define STM32_HSECLK            16000000

#define STM32F10X_CL

/*
 * GPIO
 */
// Peripheral
#define GPIO_PORT_PERIPH_RESET          GPIOB
#define GPIO_PIN_PERIPH_RESET           7

#define GPIO_PORT_BAROMETER_CHIP_SELECT GPIOA
#define GPIO_PIN_BAROMETER_CHIP_SELECT  15

#define GPIO_PORT_COMPASS_CHIP_SELECT   GPIOD
#define GPIO_PIN_COMPASS_CHIP_SELECT    2

#define GPIO_PORT_COMPASS_DATA_READY    GPIOB
#define GPIO_PIN_COMPASS_DATA_READY     6

// HWID
#define GPIO_PORT_HWID                  GPIOC
#define GPIO_PIN_HWID_BIT0              1
#define GPIO_PIN_HWID_BIT1_INVERSE      2
#define GPIO_PIN_HWID_BIT2              3

// LED
#define GPIO_PORT_LED_STATUS            GPIOB
#define GPIO_PIN_LED_STATUS             3

#define GPIO_PORT_LED_CAN1              GPIOB
#define GPIO_PIN_LED_CAN1               5

#define GPIO_PORT_LED_CAN2              GPIOB
#define GPIO_PIN_LED_CAN2               4

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

#define VAL_GPIOACRL            0x88888A48                      // 7..0
#define VAL_GPIOACRH            0x28800888                      // 15..8
#define VAL_GPIOAODR            ((1 << 13) | (1 << GPIO_PIN_BAROMETER_CHIP_SELECT))     // SWDIO pull up

#define VAL_GPIOBCRL            0x28222888
#define VAL_GPIOBCRH            0x88488A48
#define VAL_GPIOBODR            ((1 << GPIO_PIN_PERIPH_RESET))

#define VAL_GPIOCCRL            0x88888888
#define VAL_GPIOCCRH            0x888A4A88
#define VAL_GPIOCODR            ((1 << GPIO_PIN_HWID_BIT1_INVERSE))

#define VAL_GPIODCRL            0x88888288
#define VAL_GPIODCRH            0x88888888
#define VAL_GPIODODR            ((1 << GPIO_PIN_COMPASS_CHIP_SELECT))

#define VAL_GPIOECRL            0x88888888
#define VAL_GPIOECRH            0x88888888
#define VAL_GPIOEODR            0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
    void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */
