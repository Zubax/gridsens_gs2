/*
 * Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
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

#include "cli.hpp"
#include <board/board.hpp>
#include <board/usb_cdc_acm.hpp>
#include <unistd.h>
#include <cstdio>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/watchdog/watchdog.hpp>
#include <zubax_chibios/util/base64.hpp>
#include <zubax_chibios/util/shell.hpp>
#include <zubax_chibios/bootloader/loaders/ymodem.hpp>
#include <ch.hpp>
#include <hal.h>


namespace cli
{
namespace
{

static constexpr unsigned WatchdogTimeoutMSec = 3000;

static os::watchdog::Timer g_watchdog;
static os::bootloader::Bootloader* g_bootloader = nullptr;


class RebootCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "reboot"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override
    {
        os::requestReboot();
    }
} static cmd_reboot;


class ZubaxIDCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "zubax_id"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        ios.print("product_id: '%s'\n", PRODUCT_ID_STRING);
        ios.print("product_name: '%s'\n", PRODUCT_NAME_STRING);
        ios.print("mode: bootloader\n");

        ios.print("bl_version: '%u.%u'\n", BL_VERSION_MAJOR, BL_VERSION_MINOR);
        ios.print("bl_vcs_commit: %u\n", GIT_HASH);
        ios.print("bl_build_date: %s\n", __DATE__);

        {
            auto hw_version = board::detectHardwareVersion();
            ios.print("hw_version: '%u.%u'\n", hw_version.major, hw_version.minor);
        }

        {
            char base64_buf[os::base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];

            ios.print("hw_unique_id: '%s'\n", os::base64::encode(board::readUniqueID(), base64_buf));

            board::DeviceSignature signature;
            if (board::tryReadDeviceSignature(signature))
            {
                ios.print("hw_signature: '%s'\n", os::base64::encode(signature, base64_buf));
            }
        }

        ASSERT_ALWAYS(g_bootloader != nullptr);
        const auto appinfo = g_bootloader->getAppInfo();
        if (appinfo.second)
        {
            const auto& inf = appinfo.first;
            ios.print("sw_version: '%u.%u'\n", inf.major_version, inf.minor_version);
            ios.print("sw_vcs_commit: %u\n", unsigned(inf.vcs_commit));
        }
    }
} static cmd_zubax_id;


class WaitCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "wait"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override
    {
        ASSERT_ALWAYS(g_bootloader != nullptr);
        g_bootloader->cancelBoot();
    }
} static cmd_wait;


class DownloadCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "download"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        ASSERT_ALWAYS(g_bootloader != nullptr);

        os::bootloader::ymodem_loader::YModemReceiver loader(ios.getChannel(), &g_watchdog);

        g_watchdog.reset();
        int res = g_bootloader->upgradeApp(loader);
        g_watchdog.reset();

        if (res < 0)
        {
            ios.print("ERROR %d\n", res);
        }
    }
} static cmd_download;


class CLIThread : public chibios_rt::BaseStaticThread<2048>
{
    os::shell::Shell<> shell_;

    void main() override
    {
        const auto usb_port = board::usb_cdc_acm::getSerialUSBDriver();
        const auto uart_port = &STDOUT_SD;

        while (!os::isRebootRequested())
        {
            const bool using_usb = os::getStdIOStream() == reinterpret_cast<::BaseChannel*>(usb_port);
            const bool usb_connected = board::usb_cdc_acm::getState() == board::usb_cdc_acm::State::Connected;

            if (using_usb != usb_connected)
            {
                DEBUG_LOG("Switching to %s\n", usb_connected ? "USB" : "UART");
                os::setStdIOStream(usb_connected ?
                                   reinterpret_cast<::BaseChannel*>(usb_port) :
                                   reinterpret_cast<::BaseChannel*>(uart_port));
                shell_.reset();
            }

            g_watchdog.reset();

            os::shell::BaseChannelWrapper wrapper(os::getStdIOStream());
            shell_.runFor(wrapper, 100);

            g_watchdog.reset();
        }

        DEBUG_LOG("Disconnecting USB\n");
        os::setStdIOStream(reinterpret_cast<::BaseChannel*>(uart_port));
        usbDisconnectBus(board::usb_cdc_acm::getSerialUSBDriver()->config->usbp);
    }

    static os::shell::Shell<>::Prompt renderPrompt()
    {
        if (g_bootloader == nullptr)
        {
            return "> ";
        }
        else
        {
            return os::heapless::concatenate(os::bootloader::stateToString(g_bootloader->getState()), "> ");
        }
    }

public:
    CLIThread() :
        shell_(&CLIThread::renderPrompt)
    {
        shell_.addCommandHandler(&cmd_reboot);
        shell_.addCommandHandler(&cmd_zubax_id);
        shell_.addCommandHandler(&cmd_wait);
        shell_.addCommandHandler(&cmd_download);
    }

    virtual ~CLIThread() { }
} cli_thread;

} // namespace

void init(os::bootloader::Bootloader& bl)
{
    g_watchdog.startMSec(WatchdogTimeoutMSec);
    g_bootloader = &bl;
    cli_thread.start(LOWPRIO + 1);
}

}
