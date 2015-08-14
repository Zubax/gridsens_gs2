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
 * Returns version info of the currently running firmware image.
 * This function relies on the firmware image post-processing; refer to the build system docs for details.
 */
uavcan::protocol::SoftwareVersion makeUavcanSoftwareVersionStruct();

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

/**
 * Initializes the shared data structure with given values.
 */
void passParametersToBootloader(std::uint32_t can_bus_bit_rate, uavcan::NodeID node_id);

}
