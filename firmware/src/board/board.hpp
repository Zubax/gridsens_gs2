/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <cstdint>
#include <array>

namespace board
{

void init();

__attribute__((noreturn))
void die(int error);

void setCANLed(unsigned iface_index, bool state);
void setStatusLed(bool state);

void enterBootloader();

void restart();

bool isDebugSerialConnected();

typedef std::array<std::uint8_t, 12> UniqueID;
void readUniqueID(UniqueID& out_bytes);

typedef std::array<std::uint8_t, 128> DeviceSignature;
bool tryReadDeviceSignature(DeviceSignature& out_sign);
bool tryWriteDeviceSignature(const DeviceSignature& sign);

}
