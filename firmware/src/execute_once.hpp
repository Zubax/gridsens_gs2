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

#define EXECUTE_ONCE_CAT1(a, b) EXECUTE_ONCE_CAT2(a, b)
#define EXECUTE_ONCE_CAT2(a, b) a##b

/**
 * This macro can be used in function and method bodies to execute a certain block of code only once.
 * Every instantiation creates one static variable.
 * This macro is not thread safe.
 *
 * Usage:
 *   puts("Regular code");
 *   EXECUTE_ONCE_NON_THREAD_SAFE
 *   {
 *      puts("This block will be executed only once");
 *   }
 *   puts("Regular code again");
 */
#define EXECUTE_ONCE_NON_THREAD_SAFE \
    static bool EXECUTE_ONCE_CAT1(_executed_once_, __LINE__) = false; \
    for (; EXECUTE_ONCE_CAT1(_executed_once_, __LINE__) == false; EXECUTE_ONCE_CAT1(_executed_once_, __LINE__) = true)
