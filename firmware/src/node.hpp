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
