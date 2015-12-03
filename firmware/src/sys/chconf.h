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

#define CH_FREQUENCY                    1000

#define CH_USE_HEAP                     FALSE
#define CH_USE_DYNAMIC                  FALSE
#define CH_DBG_FILL_THREADS             TRUE

#if defined(DEBUG_BUILD) && DEBUG_BUILD
#   define CH_OPTIMIZE_SPEED               FALSE
#   define CH_DBG_SYSTEM_STATE_CHECK       TRUE
#   define CH_DBG_ENABLE_CHECKS            TRUE
#   define CH_DBG_ENABLE_ASSERTS           TRUE
#   define CH_DBG_ENABLE_STACK_CHECK       TRUE
#   define CH_DBG_THREADS_PROFILING        TRUE
#elif defined(RELEASE_BUILD) && RELEASE_BUILD
#   define CH_DBG_THREADS_PROFILING        FALSE
#else
#   error "Invalid configuration: Either DEBUG_BUILD or RELEASE_BUILD must be true"
#endif

#define PORT_IDLE_THREAD_STACK_SIZE    64
#define PORT_INT_REQUIRED_STACK        512

/*
 * Max line length must be large enough to accommodate signature installation command:
 *   ch> signature <172 bytes of signature>
 */
#define SHELL_MAX_LINE_LENGTH           200

#include <zubax_chibios/sys/chconf_tail.h>
