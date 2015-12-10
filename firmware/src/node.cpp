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

#include "node.hpp"

#include "board/board.hpp"
#include "component_status_manager.hpp"
#include "bootloader_interface.hpp"
#include <ch.hpp>
#include <unistd.h>

#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/param_server.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>

#include <zubax_chibios/config/config.hpp>
#include <zubax_chibios/watchdog/watchdog.hpp>
#include <zubax_chibios/sys/sys.h>

namespace node
{
namespace
{

const unsigned MinTimeSyncPubPeriodUSec = 500000;
const unsigned IfaceLedUpdatePeriodMSec = 25;

zubax_chibios::config::Param<unsigned> param_can_bitrate("uavcan.bit_rate", 0, 0, 1000000);
zubax_chibios::config::Param<unsigned> param_node_id("uavcan.node_id", 0, 0, 125);

zubax_chibios::config::Param<unsigned> param_time_sync_period_usec("uavcan.pubp-time",
                                                                   0, 0, 1000000);

zubax_chibios::config::Param<unsigned> param_time_sync_prio("uavcan.prio-time",
                                                            1,
                                                            uavcan::TransferPriority::NumericallyMin,
                                                            uavcan::TransferPriority::NumericallyMax);

zubax_chibios::config::Param<unsigned> param_node_status_pub_interval_usec(
    "uavcan.pubp-stat",
    200000,
    uavcan::protocol::NodeStatus::MIN_BROADCASTING_PERIOD_MS * 1000,
    uavcan::protocol::NodeStatus::MAX_BROADCASTING_PERIOD_MS * 1000);

zubax_chibios::config::Param<unsigned> param_node_status_prio("uavcan.prio-stat",
                                                              20,
                                                              uavcan::TransferPriority::NumericallyMin,
                                                              uavcan::TransferPriority::NumericallyMax);

uavcan_stm32::CanInitHelper<> can;
std::uint32_t active_can_bus_bit_rate;

uavcan_stm32::Mutex node_mutex;

ComponentStatusManager<unsigned(ComponentID::NumComponents_)> comp_mgr;

bool started = false;
bool pending_restart_request = false;
bool local_utc_updated = false;
bool time_sync_master_enabled = false;

void configureNode()
{
    Node& node = getNode();

    node.setName(PRODUCT_ID_STRING);

    // Software version
    node.setSoftwareVersion(bootloader_interface::makeUavcanSoftwareVersionStruct());

    // Hardware version
    uavcan::protocol::HardwareVersion hwver;
    hwver.major = HW_VERSION;

    board::UniqueID uid;
    board::readUniqueID(uid);
    std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

    board::DeviceSignature coa;
    if (board::tryReadDeviceSignature(coa))
    {
        std::copy(std::begin(coa), std::end(coa), std::back_inserter(hwver.certificate_of_authenticity));
    }

    node.setHardwareVersion(hwver);
}

void configureClockSync()
{
    // TODO: Hardware support for clock sync via PPS line
    // STM32 driver needs better clock speed adjustment algorithm
    auto params = uavcan_stm32::clock::getUtcSyncParams();

    params.offset_p = 0.0001;
    params.rate_i = 0.001;
    params.rate_error_corner_freq = 0.00005;
    params.max_rate_correction_ppm = 70;
    params.min_jump = uavcan::UtcDuration::fromMSec(300);
    params.lock_thres_offset = params.min_jump;

    uavcan_stm32::clock::setUtcSyncParams(params);
}

uavcan::ParamServer& getParamServer()
{
    static uavcan::ParamServer server(getNode());
    return server;
}

uavcan::GlobalTimeSyncMaster& getTimeSyncMaster()
{
    static uavcan::GlobalTimeSyncMaster master(getNode());
    return master;
}

uavcan::GlobalTimeSyncSlave& getTimeSyncSlave()
{
    static uavcan::GlobalTimeSyncSlave ts(getNode());
    return ts;
}

bool isLocalUtcSourceEnabled()
{
    if (getTimeSyncSlave().isActive())
    {
        return getNode().getNodeID() < getTimeSyncSlave().getMasterNodeID();
    }
    return true;
}

void publishTimeSync(const uavcan::TimerEvent&)
{
    assert(time_sync_master_enabled);
    if (isLocalUtcSourceEnabled())
    {
        getTimeSyncSlave().suppress(local_utc_updated);
    }
    else
    {
        getTimeSyncSlave().suppress(false);
    }
    local_utc_updated = false;

    if (uavcan_stm32::clock::isUtcLocked())
    {
        (void)getTimeSyncMaster().publish();
    }
}

/*
 * Param access server
 */
class ParamManager : public uavcan::IParamManager
{
    void convert(float native_value, ConfigDataType native_type, Value& out_value) const
    {
        if (native_type == CONFIG_TYPE_BOOL)
        {
            out_value.to<Value::Tag::boolean_value>() = uavcan::isCloseToZero(native_value);
        }
        else if (native_type == CONFIG_TYPE_INT)
        {
            out_value.to<Value::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
        }
        else if (native_type == CONFIG_TYPE_FLOAT)
        {
            out_value.to<Value::Tag::real_value>() = native_value;
        }
        else
        {
            ; // Invalid type
        }
    }

    void convert(float native_value, ConfigDataType native_type, NumericValue& out_value) const
    {
        if (native_type == CONFIG_TYPE_INT)
        {
            out_value.to<NumericValue::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
        }
        else if (native_type == CONFIG_TYPE_FLOAT)
        {
            out_value.to<NumericValue::Tag::real_value>() = native_value;
        }
        else
        {
            ; // Not applicable
        }
    }

    void getParamNameByIndex(Index index, Name& out_name) const override
    {
        const char* name = configNameByIndex(index);
        if (name != nullptr)
        {
            out_name = name;
        }
    }

    void assignParamValue(const Name& name, const Value& value) override
    {
        float native_value = 0.F;

        if (value.is(Value::Tag::boolean_value))
        {
            native_value = (*value.as<Value::Tag::boolean_value>()) ? 1.F : 0.F;
        }
        else if (value.is(Value::Tag::integer_value))
        {
            native_value = static_cast<float>(*value.as<Value::Tag::integer_value>());
        }
        else if (value.is(Value::Tag::real_value))
        {
            native_value = *value.as<Value::Tag::real_value>();
        }
        else
        {
            return;
        }

        (void)configSet(name.c_str(), native_value);
    }

    void readParamValue(const Name& name, Value& out_value) const override
    {
        ConfigParam descr;
        const int res = configGetDescr(name.c_str(), &descr);
        if (res >= 0)
        {
            convert(configGet(name.c_str()), descr.type, out_value);
        }
    }

    void readParamDefaultMaxMin(const Name& name, Value& out_default,
                                NumericValue& out_max, NumericValue& out_min) const override
    {
        ConfigParam descr;
        const int res = configGetDescr(name.c_str(), &descr);
        if (res >= 0)
        {
            convert(descr.default_, descr.type, out_default);
            convert(descr.max, descr.type, out_max);
            convert(descr.min, descr.type, out_min);
        }
    }

    int saveAllParams() override
    {
        return configSave();
    }

    int eraseAllParams() override
    {
        return configErase();
    }
} param_manager;

/*
 * Restart handler
 */
class RestartRequestHandler : public uavcan::IRestartRequestHandler
{
    bool handleRestartRequest(uavcan::NodeID request_source) override
    {
        lowsyslog("Restart request from %i\n", int(request_source.get()));
        pending_restart_request = true;
        return true;
    }
} restart_request_handler;

/*
 * Firmware update handler
 */
typedef uavcan::ServiceServer<uavcan::protocol::file::BeginFirmwareUpdate,
    void (*)(const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>&,
             uavcan::protocol::file::BeginFirmwareUpdate::Response&)>
    BeginFirmwareUpdateServer;

BeginFirmwareUpdateServer& getBeginFirmwareUpdateServer()
{
    static BeginFirmwareUpdateServer srv(getNode());
    return srv;
}

void handleBeginFirmwareUpdateRequest(
    const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& request,
    uavcan::protocol::file::BeginFirmwareUpdate::Response& response)
{
    static bool in_progress = false;

    ::lowsyslog("BeginFirmwareUpdate request from %d\n", int(request.getSrcNodeID().get()));

    if (in_progress)
    {
        response.error = response.ERROR_IN_PROGRESS;
    }
    else
    {
        assert(active_can_bus_bit_rate > 0);
        bootloader_interface::passParametersToBootloader(active_can_bus_bit_rate, getNode().getNodeID());

        pending_restart_request = true;
        in_progress = true;
    }
}

/*
 * UAVCAN spin loop
 */
class : public chibios_rt::BaseStaticThread<3000>
{
    mutable uavcan::MonotonicTime prev_led_update;

    void initCanBus() const
    {
        int res = 0;
        do
        {
            ::sleep(1);

            std::uint32_t bitrate = param_can_bitrate.get();
            if (bitrate == 0)
            {
                bitrate = bootloader_interface::getInheritedCanBusBitRate();
            }

            const bool autodetect = bitrate == 0;

            res = can.init([]() { ::usleep(can.getRecommendedListeningDelay().toUSec()); },
                           bitrate);

            if (res >= 0)
            {
                ::lowsyslog("CAN inited at %u bps\n", unsigned(bitrate));
                active_can_bus_bit_rate = bitrate;
            }
            else
            {
                ::lowsyslog("Could not init CAN; status: %d, autodetect: %d, bitrate: %u\n",
                            res, int(autodetect), unsigned(bitrate));
            }
        }
        while (res < 0);
    }

    int init() const
    {
        {
            Lock locker;

            configureNode();
            configureClockSync();

            // Starting the node
            const int uavcan_start_res = getNode().start(param_node_status_prio.get());
            if (uavcan_start_res < 0)
            {
                return -1000 + uavcan_start_res;
            }
            assert(getNode().isStarted());

            getNode().setRestartRequestHandler(&restart_request_handler);

            getNode().getNodeStatusProvider().setStatusPublicationPeriod(
                uavcan::MonotonicDuration::fromUSec(param_node_status_pub_interval_usec.get()));
        }

        // Configuring the local node ID
        if (param_node_id.get() > 0 || bootloader_interface::getInheritedNodeID().isUnicast())
        {
            Lock locker;
            getNode().setNodeID((param_node_id.get() > 0) ?
                                static_cast<std::uint8_t>(param_node_id.get()) :
                                bootloader_interface::getInheritedNodeID());
            lowsyslog("Using static node ID %d\n", int(getNode().getNodeID().get()));
        }
        else
        {
            uavcan::DynamicNodeIDClient dnid_client(getNode());

            {
                Lock locker;
                const int res = dnid_client.start(getNode().getNodeStatusProvider().getHardwareVersion().unique_id);
                if (res < 0)
                {
                    return -2000 + res;
                }
            }

            lowsyslog("Waiting for dynamic node ID allocation...\n");

            while (!dnid_client.isAllocationComplete())
            {
                ::usleep(1000);
                Lock locker;
                spinOnce();
            }

            lowsyslog("Dynamic node ID %d allocated by %d\n",
                      int(dnid_client.getAllocatedNodeID().get()), int(dnid_client.getAllocatorNodeID().get()));

            getNode().setNodeID(dnid_client.getAllocatedNodeID());
        }

        Lock locker;

        const int param_server_res = getParamServer().start(&param_manager);
        if (param_server_res < 0)
        {
            return -3000 + param_server_res;
        }

        const int time_sync_res = getTimeSyncSlave().start();
        if (time_sync_res < 0)
        {
            return -4000 + param_server_res;
        }

        // Time sync master - if enabled
        if (param_time_sync_period_usec.get() > 0)
        {
            time_sync_master_enabled = true;
            const int res = getTimeSyncMaster().init(param_time_sync_prio.get());
            if (res < 0)
            {
                return -5000 + res;
            }

            const auto period_usec = std::max(MinTimeSyncPubPeriodUSec, param_time_sync_period_usec.get());

            lowsyslog("Time sync enabled, period %f sec\n", period_usec * 1e-6F);

            static uavcan::Timer tsm_timer(getNode());
            tsm_timer.setCallback(&publishTimeSync);
            tsm_timer.startPeriodic(uavcan::MonotonicDuration::fromUSec(period_usec));
        }
        else
        {
            time_sync_master_enabled = false;
            lowsyslog("Time sync disabled\n");
        }

        // Firmware update server
        const int begin_firmware_update_res = getBeginFirmwareUpdateServer().start(&handleBeginFirmwareUpdateRequest);
        if (begin_firmware_update_res < 0)
        {
            return -6000 + begin_firmware_update_res;
        }

        started = true;
        lowsyslog("UAVCAN node started, ID %i\n", int(getNode().getNodeID().get()));

        return 0;
    }

    void spinOnce() const
    {
        getNode().getNodeStatusProvider().setHealth(comp_mgr.getWorstHealth());

        const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromUSec(500));
        if (spin_res < 0)
        {
            lowsyslog("UAVCAN spin failure: %i\n", spin_res);
        }

        // Iface LED update
        const auto ts = uavcan_stm32::clock::getMonotonic();
        if ((ts - prev_led_update).toMSec() >= IfaceLedUpdatePeriodMSec)
        {
            prev_led_update = ts;
            for (unsigned i = 0; i < can.driver.getNumIfaces(); i++)
            {
                board::setCANLed(i, can.driver.getIface(i)->hadActivity());
            }
        }
    }

public:
    msg_t main() override
    {
        setName("uavcan");
        initCanBus();

        assert(active_can_bus_bit_rate > 0);

        while (true)
        {
            const int res = init();
            if (res >= 0)
            {
                break;
            }
            ::lowsyslog("UAVCAN node init failed [%i], will retry\n", res);
            ::sleep(3);
        }

        zubax_chibios::watchdog::Timer wdt;
        wdt.startMSec(100);

        while (!hasPendingRestartRequest())
        {
            ::usleep(1000);
            wdt.reset();

            Lock locker;

            if (getNode().getNodeStatusProvider().getMode() != uavcan::protocol::NodeStatus::MODE_OPERATIONAL)
            {
                if (comp_mgr.areAllInitialized())
                {
                    getNode().setModeOperational();
                }
            }

            spinOnce();
        }

        lowsyslog("UAVCAN terminated\n");
        return msg_t();
    }
} node_thread;

}

Lock::Lock() : uavcan_stm32::MutexLocker(node_mutex) { }

bool isStarted()
{
    return started;
}

bool hasPendingRestartRequest()
{
    return pending_restart_request;
}

Node& getNode()
{
    static Node node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

void adjustUtcTimeFromLocalSource(const uavcan::UtcDuration& adjustment)
{
    Lock locker;
    if (time_sync_master_enabled && isLocalUtcSourceEnabled())
    {
        getTimeSyncSlave().suppress(true);
        uavcan_stm32::clock::adjustUtc(adjustment);
        local_utc_updated = true;
    }
}

void setComponentHealth(ComponentID comp, std::uint8_t health)
{
    Lock locker;
    comp_mgr.setHealth(comp, health);
}

std::uint8_t getWorstComponentHealth()
{
    Lock locker;
    return comp_mgr.getWorstHealth();
}

void markComponentInitialized(ComponentID comp)
{
    Lock locker;
    comp_mgr.markInitialized(comp);
}

void init()
{
    (void)node_thread.start(NORMALPRIO);
}

}
