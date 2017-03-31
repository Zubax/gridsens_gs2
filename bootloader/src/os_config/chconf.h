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

// There is no 32-bit timers on STM32F105
#define CH_CFG_ST_RESOLUTION            16

/**
 * High frequency is required because of the limitations of the CAN driver, refer to the CAN driver wrapper for info.
 * Frequency values that provide system tick intervals that are integer number of microseconds:
 *      1000, 1250, 1600, 2000, 2500, 3125, 4000, 5000, 6250, 8000, 10000
 */
#define CH_CFG_ST_FREQUENCY             6250

#define PORT_IDLE_THREAD_STACK_SIZE     64
#define PORT_INT_REQUIRED_STACK         512

#define CH_CFG_OPTIMIZE_SPEED           FALSE
#define CH_CFG_USE_REGISTRY             FALSE
#define CH_CFG_USE_WAITEXIT             FALSE
#define CH_CFG_USE_SEMAPHORES           FALSE
#define CH_CFG_USE_CONDVARS             FALSE
#define CH_CFG_USE_EVENTS               FALSE
#define CH_CFG_USE_QUEUES               FALSE
#define CH_CFG_USE_MEMCORE              FALSE
#define CH_DBG_STATISTICS               FALSE
#define CH_DBG_SYSTEM_STATE_CHECK       FALSE
#define CH_DBG_ENABLE_CHECKS            TRUE            // Disable later if running out of ROM
#define CH_DBG_ENABLE_ASSERTS           TRUE            // Disable later if running out of ROM
#define CH_DBG_ENABLE_STACK_CHECK       FALSE
#define CH_DBG_FILL_THREADS             FALSE

#include <zubax_chibios/sys/chconf_tail.h>
