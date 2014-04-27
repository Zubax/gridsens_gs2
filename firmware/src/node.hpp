/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#pragma once

#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/helpers/component_status_manager.hpp>

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

typedef uavcan::Node<UAVCAN_MEM_POOL_BLOCK_SIZE * 128> Node;
typedef uavcan::ComponentStatusManager<unsigned(ComponentID::NumComponents_)> ComponentStatusManager;

bool isStarted();

Node& getNode();

void adjustUtcTimeFromLocalSource(const uavcan::UtcDuration& adjustment);

void setComponentStatus(ComponentID comp, ComponentStatusManager::StatusCode status);

void init();

}
