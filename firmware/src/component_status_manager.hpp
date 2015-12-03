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

#include <cassert>
#include <algorithm>
#include <cstdint>

namespace node
{

template <unsigned NumComponents>
class ComponentStatusManager
{
    bool initialized_[NumComponents] = {};
    std::uint8_t health_[NumComponents] = {};

public:
    template <typename ID>
    void markInitialized(ID raw_id)
    {
        const auto id = unsigned(raw_id);
        assert(id < NumComponents);
        initialized_[id] = true;
    }

    bool areAllInitialized() const
    {
        return std::all_of(std::begin(initialized_), std::end(initialized_), [](bool x) { return x; });
    }

    template <typename ID>
    void setHealth(ID raw_id, std::uint8_t h)
    {
        const auto id = unsigned(raw_id);
        assert(id < NumComponents);
        health_[id] = h;
    }

    std::uint8_t getWorstHealth() const
    {
        return *std::max_element(std::begin(health_), std::end(health_));
    }
};

}
