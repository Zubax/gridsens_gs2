/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#pragma once

#include <uavcan_stm32/uavcan_stm32.hpp>

namespace node
{

struct Lock : uavcan_stm32::MutexLocker
{
    Lock();
};

typedef uavcan::Node<UAVCAN_MEM_POOL_BLOCK_SIZE * 64> Node;

bool isStarted();

Node& getNode();

int init();

}
