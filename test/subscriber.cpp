/*
 * Copyright (c) 2014 Courierdrone, courierdrone.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
 */

#include <iostream>
#include <cassert>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/equipment/airdata/StaticAirData.hpp>
#include <uavcan/equipment/airdata/AltitudeAndClimbRate.hpp>
#include "debug.hpp"


static uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid,
                                      const std::string& name)
{
    auto node = uavcan_linux::makeNode(ifaces);
    node->setNodeID(nid);
    node->setName(name.c_str());

    uavcan::NodeInitializationResult init_result;
    const int start_res = node->start(init_result);
    ENFORCE(0 == start_res);
    if (!init_result.isOk())
    {
        throw std::runtime_error("Network conflict with node " + std::to_string(init_result.conflicting_node.get()));
    }

    node->setStatusOk();
    return node;
}

template <typename DataType>
static void printMessage(const uavcan::ReceivedDataStructure<DataType>& msg)
{
    std::cout << "[" << DataType::getDataTypeFullName() << "]\n" << msg << "\n---" << std::endl;
}

template <typename DataType>
static std::shared_ptr<uavcan::Subscriber<DataType>>
makePrintingSubscriber(const uavcan_linux::NodePtr& node)
{
    return node->makeSubscriber<DataType>(&printMessage<DataType>);
}

static void runForever(const uavcan_linux::NodePtr& node)
{
    auto sub_log = makePrintingSubscriber<uavcan::protocol::debug::LogMessage>(node);
    auto sub_air_data = makePrintingSubscriber<uavcan::equipment::airdata::StaticAirData>(node);

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0)
        {
            node->logError("spin", "Error %*", res);
        }
    }
}

int main(int argc, const char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage:\n\t" << argv[0] << " <node-id> <can-iface-name-1> [can-iface-name-N...]" << std::endl;
        return 1;
    }
    const int self_node_id = std::stoi(argv[1]);
    std::vector<std::string> iface_names;
    for (int i = 2; i < argc; i++)
    {
        iface_names.emplace_back(argv[i]);
    }
    uavcan_linux::NodePtr node = initNode(iface_names, self_node_id, "com.courierdrone.gps.test");
    runForever(node);
    return 0;
}
