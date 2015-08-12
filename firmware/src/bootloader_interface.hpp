/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdint>
#include <uavcan/uavcan.hpp>

namespace bootloader_interface
{
/**
 * Reads the bootloader's shared data structure.
 * This function must be invoked before the CAN hardware is initialized.
 */
void init();

/**
 * If known, returns the bit rate value inherited from the bootloader.
 * If unknown, return zero.
 */
std::uint32_t getInheritedCanBusBitRate();

/**
 * If known, returns the node ID value inherited from the bootloader.
 * If unknown, returns an invalid node ID.
 */
uavcan::NodeID getInheritedNodeID();

}
