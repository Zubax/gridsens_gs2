/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#pragma once

namespace board
{

void init();

__attribute__((noreturn))
void die(int error);

void setCANLed(unsigned iface_index, bool state);
void setStatusLed(bool state);

}
