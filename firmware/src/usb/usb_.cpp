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

/*
 * The file is weirdly named because the build system of ChibiOS does not allow having multiple files with
 * identical names.
 */

#include "usb_cdc_acm.hpp"
#include <usb/usb.hpp>
#include <nmea/nmea.hpp>
#include <gnss.hpp>
#include <bootloader_interface/bootloader_interface.hpp>
#include <board/board.hpp>
#include <unistd.h>
#include <cstdio>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/util/base64.hpp>
#include <ch.hpp>
#include <hal.h>
#include <shell.h>
#include <chprintf.h>


namespace usb
{
namespace
{

bool g_gnss_bridge_mode = false;


void cmd_cfg(BaseSequentialStream*, int argc, char* argv[])
{
    os::config::executeCLICommand(argc, argv);
}

void cmd_reboot(BaseSequentialStream*, int, char**)
{
    ::puts("RESTART");
    os::requestReboot();
}

void cmd_bootloader(BaseSequentialStream*, int, char**)
{
    ::puts("STARTING BOOTLOADER");
    os::requestReboot();

    bootloader_interface::AppShared shared;
    shared.stay_in_bootloader = true;
    bootloader_interface::writeSharedStruct(shared);
}

void cmd_gnssbridge(BaseSequentialStream*, int, char**)
{
    g_gnss_bridge_mode = true;

    ::puts("\nRESTART TO RESUME NORMAL OPERATION\n");
    gnss::stop();
    ::sleep(1);

    SerialDriver* const gnss_port = &gnss::getSerialPort();
    SerialUSBDriver* const cli_port = usb_cdc_acm::getSerialUSBDriver();

    std::uint8_t buffer[128];

    while (usb_cdc_acm::getState() == usb_cdc_acm::State::Connected)
    {
        // GNSS --> CLI (blocking here because the latency in this direction should be minimized)
        unsigned sz = chIQReadTimeout(&gnss_port->iqueue, buffer, sizeof(buffer), MS2ST(1));
        if (sz > 0)
        {
            // Block for a brief period of time only because the CLI port may become unwriteable
            obqWriteTimeout(&cli_port->obqueue, buffer, sz, MS2ST(10));
        }

        // CLI --> GNSS
        sz = ibqReadTimeout(&cli_port->ibqueue, buffer, sizeof(buffer), TIME_IMMEDIATE);
        if (sz > 0)
        {
            // Block forever because the GNSS port is always available
            chOQWriteTimeout(&gnss_port->oqueue, buffer, sz, TIME_INFINITE);
        }
    }
}

void cmd_signature(BaseSequentialStream*, int argc, char** argv)
{
    if (argc < 1)
    {
        board::DeviceSignature sign;

        if (board::tryReadDeviceSignature(sign))
        {
            char buf[os::base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];
            std::puts(os::base64::encode(sign, buf));
        }
        else
        {
            std::puts("Error: Read failed");
        }
    }
    else
    {
        const char* const encoded = argv[0];
        board::DeviceSignature sign;

        if (!os::base64::decode(sign, encoded))
        {
            std::puts("Error: Invalid base64");
            return;
        }

        if (!board::tryWriteDeviceSignature(sign))
        {
            std::puts("Error: Write failed");
            return;
        }
    }
}

void cmd_zubax_id(BaseSequentialStream*, int, char**)
{
    const auto sw_version = bootloader_interface::getFirmwareVersion();

    printf("product_id   : '%s'\n", PRODUCT_ID_STRING);
    printf("product_name : '%s'\n", PRODUCT_NAME_STRING);

    printf("sw_version   : '%u.%u'\n", sw_version.major, sw_version.minor);
    printf("sw_vcs_commit: %lu\n", sw_version.vcs_commit);
    printf("sw_build_date: %s\n", __DATE__);

    auto hw_version = board::detectHardwareVersion();
    printf("hw_version   : '%u.%u'\n", hw_version.major, hw_version.minor);

    char base64_buf[os::base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];

    std::array<std::uint8_t, 16> uid_128;
    std::fill(std::begin(uid_128), std::end(uid_128), 0);
    {
        board::UniqueID uid;
        board::readUniqueID(uid);
        std::copy(std::begin(uid), std::end(uid), std::begin(uid_128));
    }
    printf("hw_unique_id : '%s'\n", os::base64::encode(uid_128, base64_buf));

    std::memset(&base64_buf[0], 0, sizeof(base64_buf));
    for (unsigned i = 0; i < 16; i++)
    {
        chsnprintf(&base64_buf[i * 2], 3, "%02x", uid_128[i]);
    }
    printf("hw_info_url  : http://device.zubax.com/device_info?uid=%s\n", &base64_buf[0]);

    board::DeviceSignature signature;
    if (board::tryReadDeviceSignature(signature))
    {
        printf("hw_signature : '%s'\n", os::base64::encode(signature, base64_buf));
    }
}

void cmd_threads(BaseSequentialStream*, int, char**)
{
    static const char* ThreadStateNames[] = { CH_STATE_NAMES };

    static const auto gauge_free_stack = [](const thread_t* tp)
    {
        const std::uint8_t* limit = reinterpret_cast<std::uint8_t*>(tp->p_stklimit);
        const unsigned current = reinterpret_cast<unsigned>(tp->p_ctx.r13);
        unsigned num_bytes = 0;
        while ((*limit++ == CH_DBG_STACK_FILL_VALUE) &&
               (reinterpret_cast<unsigned>(limit) < current))
        {
            num_bytes++;
        }
        return num_bytes;
    };

    puts("Name             State     FStk Prio");
    puts("-------------------------------------");
    thread_t* tp = chRegFirstThread();
    do
    {
        printf("%-16s %-9s %-4u %-4u\r\n",
               tp->p_name,
               ThreadStateNames[tp->p_state],
               gauge_free_stack(tp),
               static_cast<unsigned>(tp->p_prio));
        tp = chRegNextThread(tp);
    }
    while (tp != nullptr);
}

const ::ShellCommand HandlerTable[] =
{
    {"cfg",        &cmd_cfg},
    {"reboot",     &cmd_reboot},
    {"bootloader", &cmd_bootloader},
    {"gnssbridge", &cmd_gnssbridge},
    {"signature",  &cmd_signature},
    {"zubax_id",   &cmd_zubax_id},
    {"threads",    &cmd_threads},
    {nullptr, nullptr}
};

class USBControlThread : public chibios_rt::BaseStaticThread<1024>
{
    static bool isBaudrateValidForNMEA(unsigned baudrate)
    {
        // These are standard baud rates
        return (baudrate >= 4800) && (baudrate <= 57600);
    }

    static void initUSB()
    {
        usb_cdc_acm::DeviceSerialNumber sn;

        board::UniqueID unique_id;
        board::readUniqueID(unique_id);

        std::fill(std::begin(sn), std::end(sn), 0);
        std::copy(std::begin(unique_id), std::end(unique_id), std::begin(sn));

        usb_cdc_acm::init(sn);
    }

    void main() override
    {
        setName("usb_ctl");

        initUSB();

        ::shellInit();

        ::os::lowsyslog("USB inited\n");

        static const ShellConfig shell_config
        {
            reinterpret_cast<BaseSequentialStream*>(usb_cdc_acm::getSerialUSBDriver()),
            HandlerTable
        };

        static THD_WORKING_AREA(wa_shell, 2048);

        const auto usb_output = reinterpret_cast<::BaseChannel*>(usb_cdc_acm::getSerialUSBDriver());
        const auto uart_output = reinterpret_cast<::BaseChannel*>(&STDOUT_SD);

        thread_t* shelltp = nullptr;
        for (;;)
        {
            const bool connected = usb_cdc_acm::waitForStateChange(1000) == usb_cdc_acm::State::Connected;

            if (connected)
            {
                const auto baudrate = unsigned(usb_cdc_acm::getBaudRate());

                if (shelltp == nullptr)
                {
                    os::lowsyslog("Starting shell [br %u]\n", baudrate);
                    shelltp = shellCreateStatic(&shell_config, &wa_shell, sizeof(wa_shell), LOWPRIO);
                    os::setStdIOStream(usb_output);
                }

                const bool nmea_baudrate = isBaudrateValidForNMEA(baudrate) && !g_gnss_bridge_mode;

                if (nmea_baudrate && !nmea::hasOutput(usb_output))
                {
                    os::lowsyslog("Adding USB NMEA output [br %u]\n", baudrate);
                    nmea::addOutput(usb_output);
                }

                if (!nmea_baudrate && nmea::hasOutput(usb_output))
                {
                    os::lowsyslog("Removing USB NMEA output [br %u]\n", baudrate);
                    nmea::removeOutput(usb_output);
                }
            }
            else
            {
                if (nmea::hasOutput(usb_output))
                {
                    os::lowsyslog("Removing USB NMEA output [disconnected]\n");
                    nmea::removeOutput(usb_output);
                }
            }

            if ((shelltp != nullptr) && chThdTerminatedX(shelltp))
            {
                os::setStdIOStream(uart_output);
                shelltp = NULL;
                os::lowsyslog("Shell terminated\n");
            }
        }
    }

public:
    virtual ~USBControlThread() { }
} usb_control_thread;

} // namespace

void init()
{
    usb_control_thread.start(LOWPRIO + 1);
}

}
