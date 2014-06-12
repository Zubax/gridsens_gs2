/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#pragma once

#include "mcuconf.h"

#define HAL_USE_TM                  TRUE
#define HAL_USE_PAL                 TRUE
#define HAL_USE_ADC                 FALSE
#define HAL_USE_CAN                 FALSE
#define HAL_USE_DAC                 FALSE
#define HAL_USE_EXT                 FALSE
#define HAL_USE_GPT                 FALSE
#define HAL_USE_I2C                 TRUE
#define HAL_USE_ICU                 FALSE
#define HAL_USE_MAC                 FALSE
#define HAL_USE_MMC_SPI             FALSE
#define HAL_USE_PWM                 FALSE
#define HAL_USE_RTC                 FALSE
#define HAL_USE_SDC                 FALSE
#define HAL_USE_SERIAL              TRUE
#define HAL_USE_SERIAL_USB          FALSE
#define HAL_USE_SPI                 FALSE
#define HAL_USE_UART                FALSE
#define HAL_USE_USB                 FALSE

#define SERIAL_DEFAULT_BITRATE      115200
#define SERIAL_BUFFERS_SIZE         512

#include <crdr_chibios/sys/halconf_tail.h>
