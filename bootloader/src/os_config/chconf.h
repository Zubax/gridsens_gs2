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
#define CH_CFG_ST_FREQUENCY             2000

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
#define CH_DBG_ENABLE_CHECKS            FALSE
#define CH_DBG_ENABLE_ASSERTS           FALSE
#define CH_DBG_ENABLE_STACK_CHECK       FALSE
#define CH_DBG_FILL_THREADS             FALSE

#include <zubax_chibios/sys/chconf_tail.h>
