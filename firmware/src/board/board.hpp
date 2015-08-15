/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <cstdint>

namespace board
{

void init();

__attribute__((noreturn))
void die(int error);

void setCANLed(unsigned iface_index, bool state);
void setStatusLed(bool state);

void enterBootloader();

void restart();

constexpr unsigned UniqueIDSize = 12;

void readUniqueID(std::uint8_t bytes[UniqueIDSize]);

}
