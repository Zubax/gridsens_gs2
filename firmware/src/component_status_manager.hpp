/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
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
