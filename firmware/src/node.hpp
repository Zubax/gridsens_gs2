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
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>


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

using FirmwareUpdateRequestCallback =
    std::function<uavcan::StorageType<uavcan::protocol::file::BeginFirmwareUpdate::Response::FieldTypes::error>::Type
                  (const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>&)>;

typedef uavcan::Node<uavcan::MemPoolBlockSize * 128> Node;

bool isStarted();

Node& getNode();

std::uint32_t getCANBitRate();

void adjustUtcTimeFromLocalSource(const uavcan::UtcDuration& adjustment);

void setComponentHealth(ComponentID comp, std::uint8_t health);

std::uint8_t getWorstComponentHealth();

void markComponentInitialized(ComponentID comp);

void init(std::uint32_t bit_rate_hint,
          std::uint8_t node_id_hint,
          std::pair<std::uint8_t, std::uint8_t> firmware_version_major_minor,
          std::uint64_t firmware_image_crc64we,
          std::uint32_t firmware_vcs_commit,
          const FirmwareUpdateRequestCallback& on_firmware_update_requested);

}
