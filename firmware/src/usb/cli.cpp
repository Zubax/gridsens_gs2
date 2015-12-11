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

#include "base64.hpp"
#include "usb_cdc_acm.hpp"
#include <usb/cli.hpp>
#include <gnss.hpp>
#include <bootloader_interface.hpp>
#include <board/board.hpp>
#include <unistd.h>
#include <cstdio>
#include <zubax_chibios/sys/sys.h>
#include <zubax_chibios/sys/assert_always.h>
#include <zubax_chibios/config/cli.hpp>
#include <zubax_chibios/watchdog/watchdog.hpp>
#include <ch.hpp>
#include <hal.h>
#include <shell.h>

namespace cli
{

namespace
{

void cmd_cfg(BaseSequentialStream*, int argc, char* argv[])
{
    zubax_chibios::config::executeCliCommand(argc, argv);
}

void cmd_reboot(BaseSequentialStream*, int, char**)
{
    ::puts("RESTART");
    ::usleep(10000);
    ::NVIC_SystemReset();
}

void cmd_gnssbridge(BaseSequentialStream*, int, char**)
{
    ::puts("\nRESTART TO RESUME NORMAL OPERATION\n");
    gnss::stop();
    ::sleep(1);

    static const auto copy_once = [](InputQueue* src, OutputQueue* dst)
    {
        uint8_t buffer[128];
        const unsigned sz = chIQReadTimeout(src, buffer, sizeof(buffer), TIME_IMMEDIATE);
        if (sz > 0)
        {
            chOQWriteTimeout(dst, buffer, sz, TIME_INFINITE);
        }
    };

    SerialDriver* const gnss_port = &gnss::getSerialPort();
    SerialUSBDriver* const cli_port  = usb_cdc_acm::getSerialUSBDriver();

    while (usb_cdc_acm::isConnected())
    {
        copy_once(&gnss_port->iqueue, &cli_port->oqueue);
        copy_once(&cli_port->iqueue, &gnss_port->oqueue);
    }
}

void cmd_bootloader(BaseSequentialStream*, int, char**)
{
    ::puts("\nENTERING THE BOOTLOADER\n");
    ::usleep(100000);

    // Suppress the watchdog - set the maximum possible interval
    zubax_chibios::watchdog::Timer().startMSec(1000000);

    board::enterBootloader();
}

void cmd_signature(BaseSequentialStream*, int argc, char** argv)
{
    if (argc < 1)
    {
        board::DeviceSignature sign;

        if (board::tryReadDeviceSignature(sign))
        {
            char buf[base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];
            std::puts(base64::encode(sign, buf));
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

        if (!base64::decode(sign, encoded))
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
    const auto sw_version = bootloader_interface::makeUavcanSoftwareVersionStruct();

    printf("product_id   : '%s'\n", PRODUCT_ID_STRING);
    printf("product_name : '%s'\n", PRODUCT_NAME_STRING);

    printf("sw_version   : '%u.%u'\n", sw_version.major, sw_version.minor);
    printf("sw_vcs_commit: %lu\n", sw_version.vcs_commit);
    printf("sw_build_date: %s\n", __DATE__);

    auto hw_version = board::detectHardwareVersion();
    printf("hw_version   : '%u.%u'\n", hw_version.major, hw_version.minor);

    char base64_buf[base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];

    std::array<std::uint8_t, 16> uid_128;
    std::fill(std::begin(uid_128), std::end(uid_128), 0);
    {
        board::UniqueID uid;
        board::readUniqueID(uid);
        std::copy(std::begin(uid), std::end(uid), std::begin(uid_128));
    }
    printf("hw_unique_id : '%s'\n", base64::encode(uid_128, base64_buf));

    board::DeviceSignature signature;
    if (board::tryReadDeviceSignature(signature))
    {
        printf("hw_signature : '%s'\n", base64::encode(signature, base64_buf));
    }
}

#if defined(DEBUG_BUILD) && DEBUG_BUILD

void cmd_threads(BaseSequentialStream*, int, char**)
{
    static const char* ThreadStateNames[] = { THD_STATE_NAMES };

    static const auto gauge_free_stack = [](const Thread* tp)
    {
        const std::uint8_t* limit = reinterpret_cast<std::uint8_t*>(tp->p_stklimit);
        const unsigned current = reinterpret_cast<unsigned>(tp->p_ctx.r13);
        unsigned num_bytes = 0;
        while ((*limit++ == CH_STACK_FILL_VALUE) &&
               (reinterpret_cast<unsigned>(limit) < current))
        {
            num_bytes++;
        }
        return num_bytes;
    };

    puts("Name             State     FStk Prio Time");
    puts("--------------------------------------------");
    Thread* tp = chRegFirstThread();
    do
    {
        printf("%-16s %-9s %-4u %-4u %u\r\n",
               tp->p_name,
               ThreadStateNames[tp->p_state],
               gauge_free_stack(tp),
               static_cast<unsigned>(tp->p_prio),
               static_cast<unsigned>(tp->p_time));
        tp = chRegNextThread(tp);
    }
    while (tp != nullptr);
}

#endif // defined(DEBUG_BUILD) && DEBUG_BUILD

const ::ShellCommand HandlerTable[] =
{
    {"cfg",        &cmd_cfg},
    {"reboot",     &cmd_reboot},
    {"gnssbridge", &cmd_gnssbridge},
    {"bootloader", &cmd_bootloader},
    {"signature",  &cmd_signature},
    {"zubax_id",   &cmd_zubax_id},
#if defined(DEBUG_BUILD) && DEBUG_BUILD
    {"threads",    &cmd_threads},
#endif
    {nullptr, nullptr}
};

class : public chibios_rt::BaseStaticThread<1024>
{
    static void initUSB()
    {
        usb_cdc_acm::DeviceSerialNumber sn;

        board::UniqueID unique_id;
        board::readUniqueID(unique_id);

        std::fill(std::begin(sn), std::end(sn), 0);
        std::copy(std::begin(unique_id), std::end(unique_id), std::begin(sn));

        usb_cdc_acm::init(sn);
    }

public:
    msg_t main() override
    {
        setName("cli_ctl");

        initUSB();

        ::shellInit();

        ::lowsyslog("USB & CLI inited\n");

        static const ShellConfig shell_config
        {
            reinterpret_cast<BaseSequentialStream*>(usb_cdc_acm::getSerialUSBDriver()),
            HandlerTable
        };

        static WORKING_AREA(wa_shell, 2048);

        Thread* shelltp = nullptr;
        for (;;)
        {
            ::sleep(1);

            if ((shelltp == nullptr) && usb_cdc_acm::isConnected())
            {
                ::lowsyslog("Starting shell\n");
                shelltp = shellCreateStatic(&shell_config, &wa_shell, sizeof(wa_shell), LOWPRIO);
                sysSetStdOutStream(shell_config.sc_channel);
            }

            if ((shelltp != nullptr) && chThdTerminated(shelltp))
            {
                sysSetStdOutStream(reinterpret_cast<BaseSequentialStream*>(&STDOUT_SD));
                shelltp = NULL;
                ::lowsyslog("Shell terminated\n");
            }
        }

        return 0;
    }
} cli_control_thread;

} // namespace

void init()
{
    cli_control_thread.start(LOWPRIO + 1);
}

}
