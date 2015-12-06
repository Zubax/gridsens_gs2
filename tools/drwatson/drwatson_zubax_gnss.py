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

import numpy
import tempfile
import logging
import time
import uavcan
import uavcan.monitors
from contextlib import closing, contextmanager
from functools import partial
from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort, glob_one, download_newest, open_serial_port,\
    enforce


PRODUCT_NAME = 'com.zubax.gnss'
DEFAULT_FIRMWARE_GLOB = 'https://files.zubax.com/products/%s/*.compound.bin' % PRODUCT_NAME
CAN_BITRATE = 1000000
FLASH_OFFSET = 0x08000000
TOOLCHAIN_PREFIX = 'arm-none-eabi-'
DEBUGGER_PORT_GDB_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if00'
DEBUGGER_PORT_CLI_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if0[12]'
BOOT_TIMEOUT = 9
GNSS_FIX_TIMEOUT = 60 * 10
GNSS_MIN_SAT_TIMEOUT = 60 * 5
GNSS_MIN_SAT_NUM = 6


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

        runtc('gdb %s --batch -x %s -return-child-result -silent', fn('output.elf'), fn('script.gdb'))


def wait_for_boot():
    deadline = time.monotonic() + BOOT_TIMEOUT

    with open_serial_port(DEBUGGER_PORT_CLI_GLOB, timeout=BOOT_TIMEOUT) as p:
        try:
            p.flushInput()
            for line in p:
                if b'Zubax GNSS' in line:
                    return
                print('Debug output:', line)
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


def test_uavcan():
    node_info = uavcan.protocol.GetNodeInfo.Response()  # @UndefinedVariable
    node_info.name.encode('com.zubax.drwatson.zubax_gnss')

    with closing(uavcan.make_node(args.iface, bitrate=CAN_BITRATE, node_id=127,
                                  mode=uavcan.protocol.NodeStatus().MODE_OPERATIONAL)) as n:  # @UndefinedVariable
        def safe_spin(timeout):
            try:
                n.spin(timeout)
            except uavcan.UAVCANException:
                logger.error('Node spin failure', exc_info=True)

        @contextmanager
        def time_limit(timeout, error_fmt, *args):
            aborter = n.defer(timeout, partial(abort, error_fmt, *args))
            yield
            aborter.remove()

        # Dynamic node ID allocation
        try:
            nsmon = uavcan.monitors.NodeStatusMonitor(n)
            alloc = uavcan.monitors.DynamicNodeIDServer(n, nsmon)  # @UnusedVariable

            with time_limit(10, 'The node did not show up in time'):
                while True:
                    safe_spin(1)
                    target_nodes = list(nsmon.find_all(lambda e: e.info and e.info.name.decode() == PRODUCT_NAME))
                    if len(target_nodes) == 1:
                        break
                    if len(target_nodes) > 1:
                        abort('Expected to find exactly one target node, found more: %r', target_nodes)

            node_id = target_nodes[0].node_id
            info('Node %r initialized', node_id)
            print(target_nodes[0])

            def request(what):
                response_event = None

                def cb(e):
                    nonlocal response_event
                    if not e:
                        abort('Request has timed out: %r', what)
                    response_event = e  # @UnusedVariable

                n.request(what, node_id, cb)
                while response_event is None:
                    safe_spin(0.1)
                return response_event.response

            # Starting the node and checking its self-reported diag outputs
            def wait_for_init():
                with time_limit(10, 'The node did not complete initialization in time'):
                    while True:
                        safe_spin(1)
                        if nsmon.exists(node_id) and nsmon.get(node_id).status.mode == \
                                uavcan.protocol.NodeStatus().MODE_OPERATIONAL:              # @UndefinedVariable
                            break

            def check_status():
                status = nsmon.get(node_id).status
                enforce(status.mode == uavcan.protocol.NodeStatus().MODE_OPERATIONAL,   # @UndefinedVariable
                        'Unexpected operating mode')
                enforce(status.health == uavcan.protocol.NodeStatus().HEALTH_OK,        # @UndefinedVariable
                        'Bad node health')

            info('Waiting for the node to complete initialization...')
            wait_for_init()
            check_status()

            info('Reconfiguring the node...')

            def print_all_params():
                for index in range(10000):
                    r = request(uavcan.protocol.param.GetSet.Request(index=index))  # @UndefinedVariable
                    if not r.name:
                        break
                    print('Param %-30r %r' % (r.name.decode(), getattr(r.value, r.value.union_field)))

            def set_param(name, value, union_field=None):
                union_field = union_field or {
                    int: 'integer_value',
                    float: 'real_value',
                    bool: 'boolean_value',
                    str: 'string_value'
                }[type(value)]
                logger.info('Setting parameter %r field %r value %r', name, union_field, value)
                req = uavcan.protocol.param.GetSet.Request()                            # @UndefinedVariable
                req.name.encode(name)
                setattr(req.value, union_field, value)
                r = request(req)
                enforce(r.value.union_field == union_field,
                        'Union field mismatch in param set response for %r', name)
                enforce(getattr(r.value, union_field) == value,
                        'The node refused to set parameter %r', name)

            set_param('uavcan.pubp-time', 10000)
            set_param('uavcan.pubp-stat', 2000)
            set_param('uavcan.pubp-pres', 10000)
            set_param('uavcan.pubp-mag', 20000)
            set_param('uavcan.pubp-fix', 66666)
            set_param('uavcan.pubp-aux', 100000)

            enforce(request(uavcan.protocol.param.ExecuteOpcode.Request(                # @UndefinedVariable
                opcode=uavcan.protocol.param.ExecuteOpcode.Request().OPCODE_SAVE)).ok,  # @UndefinedVariable
                'Could not save configuration')

            enforce(request(uavcan.protocol.RestartNode.Request(                        # @UndefinedVariable
                magic_number=uavcan.protocol.RestartNode.Request().MAGIC_NUMBER)).ok,   # @UndefinedVariable
                'Could not restart the node')

            wait_for_boot()
            wait_for_init()
            check_status()
            print_all_params()

            def make_collector(data_type, timeout=0.1):
                return uavcan.monitors.MessageCollector(n, data_type, timeout=timeout)

            col_fix = make_collector(uavcan.equipment.gnss.Fix, 0.2)                    # @UndefinedVariable
            col_mag = make_collector(uavcan.equipment.ahrs.MagneticFieldStrength)       # @UndefinedVariable
            col_pressure = make_collector(uavcan.equipment.air_data.StaticPressure)     # @UndefinedVariable
            col_temp = make_collector(uavcan.equipment.air_data.StaticTemperature)      # @UndefinedVariable

            def check_everything():
                check_status()

                m = col_temp[node_id].message
                if not 10 < (m.static_temperature - 273.15) < 50:
                    abort('Invalid temperature reading: %d Kelvin. Check the sensor.', m.static_temperature)

                m = col_pressure[node_id].message
                if not 50000 < m.static_pressure < 150000:
                    abort('Invalid pressure reading: %d Pascal. Check the sensor.', m.static_pressure)

                m = col_mag[node_id].message
                magnetic_field_scalar = numpy.linalg.norm(m.magnetic_field_ga)          # @UndefinedVariable
                if not 0.01 < magnetic_field_scalar < 2:
                    abort('Invalid magnetic field strength reading: %d Gauss. Check the sensor.',
                          magnetic_field_scalar)

            info('Waiting for GNSS fix...')
            with time_limit(GNSS_FIX_TIMEOUT, 'GNSS fix timeout. Check the RF circuit, AFE, antenna, and receiver'):
                while True:
                    safe_spin(1)
                    check_everything()
                    if col_fix[node_id].message.status >= 3:
                        break

            info('Waiting for %d satellites...', GNSS_MIN_SAT_NUM)
            with time_limit(GNSS_MIN_SAT_TIMEOUT,
                            'GNSS performance is degraded. Could be caused by poor assembly of the RF circuit'):
                while True:
                    safe_spin(0.5)
                    check_everything()
                    num = col_fix[node_id].message.sats_used
                    sys.stdout.write('\rsats_used=%d\r' % num)
                    sys.stdout.flush()
                    if num >= GNSS_MIN_SAT_NUM:
                        break

            check_everything()

            # Finalizing the test
            info('Resetting the configuration to factory default...')
            enforce(request(uavcan.protocol.param.ExecuteOpcode.Request(                # @UndefinedVariable
                opcode=uavcan.protocol.param.ExecuteOpcode.Request().OPCODE_ERASE)).ok,  # @UndefinedVariable
                'Could not erase configuration')

            enforce(request(uavcan.protocol.RestartNode.Request(                        # @UndefinedVariable
                magic_number=uavcan.protocol.RestartNode.Request().MAGIC_NUMBER)).ok,   # @UndefinedVariable
                'Could not restart the node')

            wait_for_boot()
            wait_for_init()
            check_status()
            print_all_params()
        except Exception:
            for nid in nsmon.get_all_node_id():
                print('Node state: %r' % nsmon.get(nid))
            raise

    # Blocking questions are moved out of the node scope because blocking breaks CAN communications
    if not input('Is the PPS LED blinking once per second?', yes_no=True, default_answer=True):
        abort('PPS LED is not working')

    if not input('Is the CAN1 LED glowing solid?', yes_no=True, default_answer=True):
        abort('CAN1 LED is not working (however the interface is fine)')

    if not input('Is the STATUS LED blinking once per second?', yes_no=True, default_answer=True):
        abort('STATUS LED is not working')

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
    with CLIWaitCursor():
        pass  # load_firmware(firmware_data)

    info('Waiting for the board to boot...')
    # wait_for_boot()
    info('Booted successfully')

    if use_socketcan:
        execute_shell_command('ifconfig %s down && ifconfig %s up', args.iface, args.iface)

    info('Testing UAVCAN interface...')
    test_uavcan()


run(process_one_device)
