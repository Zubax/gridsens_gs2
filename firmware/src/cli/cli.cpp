/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "base64.hpp"
#include "usb_cli.hpp"
#include <cli/cli.hpp>
#include <gnss.hpp>
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

void cmd_reset(BaseSequentialStream*, int, char**)
{
    ::puts("RESTART");
    ::usleep(10000);
    ::NVIC_SystemReset();
}

void cmd_gnssbridge(BaseSequentialStream*, int, char**)
{
    ::puts("\nRESET THE BOARD TO RESUME NORMAL OPERATION\n");
    gnss::stop();
    ::sleep(1);

    SerialDriver& gnss_port = gnss::getSerialPort();
    SerialDriver& cli_port = STDOUT_SD;

    static const auto copy_once = [](SerialDriver* src, SerialDriver* dst)
    {
        uint8_t buffer[128];
        const unsigned sz = sdReadTimeout(src, buffer, sizeof(buffer), TIME_IMMEDIATE);
        if (sz > 0)
        {
            sdWrite(dst, buffer, sz);
        }
    };

    while (true)
    {
        copy_once(&gnss_port, &cli_port);
        copy_once(&cli_port, &gnss_port);
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

const ::ShellCommand HandlerTable[] =
{
    {"cfg",        &cmd_cfg},
    {"reset",      &cmd_reset},
    {"gnssbridge", &cmd_gnssbridge},
    {"bootloader", &cmd_bootloader},
    {"signature",  &cmd_signature},
    {"threads",    &cmd_threads},
    {nullptr, nullptr}
};

class : public chibios_rt::BaseStaticThread<1024>
{
    static void initUSB()
    {
        usb_cli::DeviceSerialNumber sn;

        board::UniqueID unique_id;
        board::readUniqueID(unique_id);

        std::fill(std::begin(sn), std::end(sn), 0);
        std::copy(std::begin(unique_id), std::end(unique_id), std::begin(sn));

        usb_cli::init(sn);
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
            reinterpret_cast<BaseSequentialStream*>(usb_cli::getSerialUSBDriver()),
            HandlerTable
        };

        static WORKING_AREA(wa_shell, 2048);

        Thread* shelltp = nullptr;
        for (;;)
        {
            ::sleep(1);

            if ((shelltp == nullptr) && usb_cli::isConnected())
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
