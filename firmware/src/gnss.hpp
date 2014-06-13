/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#pragma once

#include <ch.hpp>
#include <hal.h>

namespace gnss
{

void init();

void stop();

SerialDriver& getSerialPort();

}
