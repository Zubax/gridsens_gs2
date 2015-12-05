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

import uavcan
from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort, glob_one, download_newest

PRODUCT_NAME = 'com.zubax.gnss'
DEFAULT_FIRMWARE_GLOB = 'https://files.zubax.com/products/%s/*.compound.bin' % PRODUCT_NAME
CAN_BITRATE = 1000000


args = init('''Zubax GNSS production testing application.
Usage instructions:
    1. Connect a CAN adapter to this computer.
    2. Connect exactly one DroneCode Probe to this computer.
    3. Start this application and follow its instructions.''',
            lambda p: p.add_argument('iface', help='CAN interface or device path, e.g. "can0", "/dev/ttyACM0", etc.'),
            lambda p: p.add_argument('--firmware', '-f', help='location of the firmware file (if not provided, ' +
                                     'the firmware will be downloaded from Zubax Robotics file server)'),
            require_root=True)

dcp_port_gdb = glob_one('/dev/serial/by-id/*Black_Magic_Probe*-if00')
dcp_port_cli = glob_one('/dev/serial/by-id/*Black_Magic_Probe*-if0[12]')

use_socketcan = '/' not in args.iface

api = make_api_context_with_user_provided_credentials()


def get_firmware():
    if args.firmware:
        img = download(args.firmware)
    else:
        img = download_newest(DEFAULT_FIRMWARE_GLOB)
    assert 30 < (len(img) / 1024) <= 240, 'Invalid firmware size'
    return img


with CLIWaitCursor():
    # Downloading the firmware file
    firmware = get_firmware()

    # Initializing the CAN interface. If we're using SLCAN, entire initialization will be done by pyuavcan.
    if use_socketcan:
        execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                              args.iface, args.iface, CAN_BITRATE, ignore_failure=True)
