#!/usr/bin/env python3
#
# Copyright (C) 2015 Zubax Robotics <info@zubax.com>
#
# This program is free software: you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import sys
sys.path.insert(1, os.path.join(sys.path[0], 'pyuavcan'))

import tempfile
import logging
import time
import uavcan
from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort, glob_one, download_newest, open_serial_port


PRODUCT_NAME = 'com.zubax.gnss'
DEFAULT_FIRMWARE_GLOB = 'https://files.zubax.com/products/%s/*.compound.bin' % PRODUCT_NAME
CAN_BITRATE = 1000000
FLASH_OFFSET = 0x08000000
TOOLCHAIN_PREFIX = 'arm-none-eabi-'
DEBUGGER_PORT_GDB_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if00'
DEBUGGER_PORT_CLI_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if0[12]'
BOOT_TIMEOUT = 9


logger = logging.getLogger(__name__)


args = init('''Zubax GNSS production testing application.
Usage instructions:
    1. Connect a CAN adapter to this computer.
    2. Connect exactly one DroneCode Probe to this computer.
    3. Start this application and follow its instructions.''',
            lambda p: p.add_argument('iface', help='CAN interface or device path, e.g. "can0", "/dev/ttyACM0", etc.'),
            lambda p: p.add_argument('--firmware', '-f', help='location of the firmware file (if not provided, ' +
                                     'the firmware will be downloaded from Zubax Robotics file server)'),
            require_root=True)

use_socketcan = '/' not in args.iface

#api = make_api_context_with_user_provided_credentials()


def get_firmware():
    if args.firmware:
        img = download(args.firmware)
    else:
        img = download_newest(DEFAULT_FIRMWARE_GLOB)
    assert 30 < (len(img) / 1024) <= 240, 'Invalid firmware size'
    return img


def load_firmware(firmware_data):
    with tempfile.TemporaryDirectory('-drwatson') as tmpdir:
        logger.debug('Executable scratchpad directory: %r', tmpdir)
        fn = lambda x: os.path.join(tmpdir, x)
        runtc = lambda fmt, *a, **kw: execute_shell_command(TOOLCHAIN_PREFIX + fmt, *a, **kw)

        # Generating ELF from the downloaded binary
        with open(fn('fw.bin'), 'wb') as f:
            f.write(firmware_data)

        with open(fn('link.ld'), 'w') as f:
            f.write('SECTIONS { . = %s; .text : { *(.text) } }' % FLASH_OFFSET)

        runtc('ld -b binary -r -o %s %s', fn('tmp.elf'), fn('fw.bin'))
        runtc('objcopy --rename-section .data=.text --set-section-flags .data=alloc,code,load %s', fn('tmp.elf'))
        runtc('ld %s -T %s -o %s', fn('tmp.elf'), fn('link.ld'), fn('output.elf'))

        # Loading the ELF onto the target
        with open(fn('script.gdb'), 'w') as f:
            f.write('\n'.join([
                'target extended-remote %s' % glob_one(DEBUGGER_PORT_GDB_GLOB),
                'mon swdp_scan',
                'attach 1',
                'load',
                'compare-sections',
                'kill',
                'quit 0'
            ]))

        runtc('gdb %s --batch -x %s -return-child-result', fn('output.elf'), fn('script.gdb'))


def wait_for_boot():
    deadline = time.monotonic() + BOOT_TIMEOUT

    with open_serial_port(DEBUGGER_PORT_CLI_GLOB, timeout=BOOT_TIMEOUT) as p:
        try:
            p.flushInput()
            for line in p:
                if b'Zubax GNSS' in line:
                    return
                print(line)
                if time.monotonic() > deadline:
                    break
        except IOError:
            logging.info('Boot error', exc_info=True)
        finally:
            p.flushInput()

    error('The board did not report to CLI with a correct boot message. Possible reasons:\n'
          '1. The board could not boot properly (however it was flashed successfully).\n'
          '2. The debug connector is not soldered properly.\n'
          '3. The serial port is open by another application.')
    abort('Boot error')


with CLIWaitCursor():
    firmware_data = get_firmware()

    # Initializing the CAN interface. If we're using SLCAN, entire initialization will be done by pyuavcan.
    if use_socketcan:
        execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                              args.iface, args.iface, CAN_BITRATE, ignore_failure=True)


def process_one_device():
    input('1. Connect DroneCode Probe to the debug connector\n'
          '2. Connect CAN to the first CAN1 connector on the device; terminate the other CAN1 connector\n'
          '3. Connect USB to the device, and make sure that no other Zubax GNSS is connected\n'
          '4. Press ENTER')

    info('Loading the firmware')
    load_firmware(firmware_data)

    info('Waiting for the board to boot...')
    wait_for_boot()
    info('Booted successfully')

    if use_socketcan:
        execute_shell_command('ifconfig %s down && ifconfig %s up', args.iface, args.iface)


run(process_one_device)
