/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <uavcan_stm32/uavcan_stm32.hpp>

namespace node
{

enum class ComponentID
{
    Gnss,
    AirSensor,
    Magnetometer,
    NumComponents_
};

struct Lock : uavcan_stm32::MutexLocker
{
    Lock();
};

typedef uavcan::Node<uavcan::MemPoolBlockSize * 128> Node;

bool isStarted();

bool hasPendingRestartRequest();

Node& getNode();

void adjustUtcTimeFromLocalSource(const uavcan::UtcDuration& adjustment);

void setComponentHealth(ComponentID comp, std::uint8_t health);

void markComponentInitialized(ComponentID comp);

void init();

}
