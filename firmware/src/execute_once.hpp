/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
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
