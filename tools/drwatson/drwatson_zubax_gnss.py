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

from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort, glob_one, download_newest, open_serial_port,\
    enforce, SerialCLI, BackgroundSpinner, fatal, warning, BackgroundDelay, imperative
import numpy
import tempfile
import logging
import time
import binascii
import serial
import uavcan  # @UnusedImport
from base64 import b64decode, b64encode
from contextlib import closing, contextmanager
from functools import partial


PRODUCT_NAME = 'com.zubax.gnss'
DEFAULT_FIRMWARE_GLOB = 'https://files.zubax.com/products/%s/*.compound.bin' % PRODUCT_NAME
CAN_BITRATE = 250000            # High values lead to issues with certain CAN adapters
FLASH_OFFSET = 0x08000000
TOOLCHAIN_PREFIX = 'arm-none-eabi-'
DEBUGGER_PORT_GDB_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if00'
DEBUGGER_PORT_CLI_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if0[12]'
USB_CDC_ACM_GLOB = '/dev/serial/by-id/*Zubax_GNSS*-if00'
BOOT_TIMEOUT = 9
# GNSS constants are very pessimistic, because they largely depend on the environment where devices are tested.
GNSS_FIX_TIMEOUT = 60 * 10
GNSS_MIN_SAT_TIMEOUT = 60 * 15
GNSS_MIN_SAT_NUM = 6


logger = logging.getLogger('main')


args = init('''Zubax GNSS production testing application.
If you're a licensed manufacturer, you should have received usage
instructions with the manufacturing doc pack.''',
            lambda p: p.add_argument('iface', help='CAN interface or device path, e.g. "can0", "/dev/ttyACM0", etc.'),
            lambda p: p.add_argument('--firmware', '-f', help='location of the firmware file (if not provided, ' +
                                     'the firmware will be downloaded from Zubax Robotics file server)'),
            require_root=True)

info('''
Usage instructions:

1. Connect a CAN adapter to this computer. Supported adapters are:
1.1. SLCAN-compliant adapters. If you're using an SLCAN adapter,
     use its serial port name as CAN interface name (e.g. "/dev/ttyACM0").
1.2. SocketCAN-compatible adapters. In this case it is recommended to use
     8devices USB2CAN. Correct interface name would be "can0".

2. Connect exactly one DroneCode Probe to this computer.
   For more info refer to https://kb.zubax.com/x/iIAh.

3. Follow the instructions printed in green. If you have any questions,
   don't hesitate to reach licensing@zubax.com, or use the emergency
   contacts provided to you earlier.
''')


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

    def handle_serial_port_hanging():
        fatal('DRWATSON HAS DETECTED A PROBLEM WITH CONNECTED HARDWARE AND NEEDS TO TERMINATE.\n'
              'A serial port operation has timed out. This usually indicates a problem with the connected '
              'hardware or its drivers. Please disconnect all USB devices currently connected to this computer, '
              "then connect them back and restart Drwatson. If you're using a virtual machine, please reboot it.",
              use_abort=True)

    with BackgroundDelay(BOOT_TIMEOUT * 5, handle_serial_port_hanging):
        with open_serial_port(DEBUGGER_PORT_CLI_GLOB, timeout=BOOT_TIMEOUT) as p:
            try:
                for line in p:
                    if b'zubax gnss' in line.lower() and b'bootloader' not in line.lower():
                        logging.info('Boot detected, waiting a few seconds...')
                        time.sleep(4)
                        return
                    logger.info('Debug UART output: %s', line)
                    if time.monotonic() > deadline:
                        break
            except serial.serialutil.SerialException:
                raise
            except IOError:
                logging.info('Boot error', exc_info=True)
            finally:
                p.flushInput()

    warning("The device did not report to CLI with a correct boot message, but we're going "
            "to continue anyway. Possible reasons for this warning:\n"
            '1. The device could not boot properly (however it was flashed successfully).\n'
            '2. The debug connector is not soldered properly.\n'
            '3. The serial port is open by another application.\n'
            '4. Either USB-UART adapter or VM are malfunctioning. Try to re-connect the '
            'adapter (disconnect from USB and from the device!) or reboot the VM.')


def test_uavcan():
    node_info = uavcan.protocol.GetNodeInfo.Response()
    node_info.name = 'com.zubax.drwatson.zubax_gnss'

    iface = init_can_iface()

    with closing(uavcan.make_node(iface,
                                  bitrate=CAN_BITRATE,
                                  node_id=127,
                                  mode=uavcan.protocol.NodeStatus().MODE_OPERATIONAL,
                                  node_info=node_info)) as n:
        def safe_spin(timeout):
            try:
                n.spin(timeout)
            except uavcan.transport.TransferError:
                logger.debug('Transfer error while spinning the node. '
                             'Reporting at the DEBUG level because of https://github.com/UAVCAN/pyuavcan/issues/14',
                             exc_info=True)
            except uavcan.UAVCANException:
                logger.error('Node spin failure', exc_info=True)

        @contextmanager
        def time_limit(timeout, error_fmt, *args):
            aborter = n.defer(timeout, partial(abort, error_fmt, *args))
            yield
            aborter.remove()

        # Dynamic node ID allocation
        try:
            nsmon = uavcan.app.node_monitor.NodeMonitor(n)
            alloc = uavcan.app.dynamic_node_id.CentralizedServer(n, nsmon)

            with time_limit(10, 'The node did not show up in time. Check CAN interface and crystal oscillator.'):
                while True:
                    safe_spin(1)
                    target_nodes = list(nsmon.find_all(lambda e: e.info and e.info.name.decode() == PRODUCT_NAME))
                    if len(target_nodes) == 1:
                        break
                    if len(target_nodes) > 1:
                        abort('Expected to find exactly one target node, found more: %r', target_nodes)

            node_id = target_nodes[0].node_id
            info('Node %r initialized', node_id)
            for nd in target_nodes:
                logger.info('Discovered node %r', nd)

            def request(what, fire_and_forget=False, timeout=2):
                response_event = None

                def cb(e):
                    nonlocal response_event
                    if not e:
                        abort('Request has timed out: %r', what)
                    response_event = e

                if fire_and_forget:
                    n.request(what, node_id, lambda _: None, timeout=timeout)
                    safe_spin(0.1)
                else:
                    n.request(what, node_id, cb, timeout=timeout)
                    while response_event is None:
                        safe_spin(0.1)
                    return response_event.response

            # Starting the node and checking its self-reported diag outputs
            def wait_for_init():
                with time_limit(12, 'The node did not complete initialization in time'):
                    while True:
                        safe_spin(1)
                        if nsmon.exists(node_id) and nsmon.get(node_id).status.mode == \
                                uavcan.protocol.NodeStatus().MODE_OPERATIONAL:
                            break

            def check_status():
                status = nsmon.get(node_id).status
                enforce(status.mode == uavcan.protocol.NodeStatus().MODE_OPERATIONAL,
                        'Unexpected operating mode')
                enforce(status.health == uavcan.protocol.NodeStatus().HEALTH_OK,
                        'Bad node health')

            info('Waiting for the node to complete initialization...')
            wait_for_init()
            check_status()

            info('Reconfiguring the node...')

            def log_all_params():
                for index in range(10000):
                    r = request(uavcan.protocol.param.GetSet.Request(index=index))
                    if not r.name:
                        break
                    logger.info('Param %-30r %r' % (r.name.decode(),
                                                    getattr(r.value, uavcan.get_active_union_field(r.value))))

            def set_param(name, value, union_field=None):
                union_field = union_field or {
                    int: 'integer_value',
                    float: 'real_value',
                    bool: 'boolean_value',
                    str: 'string_value'
                }[type(value)]
                logger.info('Setting parameter %r field %r value %r', name, union_field, value)
                req = uavcan.protocol.param.GetSet.Request()
                req.name.encode(name)
                setattr(req.value, union_field, value)
                r = request(req)
                enforce(uavcan.get_active_union_field(r.value) == union_field,
                        'Union field mismatch in param set response for %r', name)
                enforce(getattr(r.value, union_field) == value,
                        'The node refused to set parameter %r', name)

            set_param('uavcan.pubp-time', 10000)
            set_param('uavcan.pubp-stat', 2000)
            set_param('uavcan.pubp-pres', 10000)
            set_param('uavcan.pubp-mag', 20000)
            set_param('uavcan.pubp-fix', 66666)
            set_param('uavcan.pubp-aux', 100000)

            enforce(request(uavcan.protocol.param.ExecuteOpcode.Request(
                opcode=uavcan.protocol.param.ExecuteOpcode.Request().OPCODE_SAVE)).ok,
                'Could not save configuration')

            request(uavcan.protocol.RestartNode.Request(
                magic_number=uavcan.protocol.RestartNode.Request().MAGIC_NUMBER),
                fire_and_forget=True)

            # Don't tell anybody I wrote this. I'm ashamed of this shit and too tired to redesign it. :(
            with BackgroundSpinner(safe_spin, 0.1):
                wait_for_boot()

            wait_for_init()
            check_status()
            log_all_params()

            def make_collector(data_type, timeout=0.5):
                return uavcan.app.message_collector.MessageCollector(n, data_type, timeout=timeout)

            col_fix = make_collector(uavcan.equipment.gnss.Fix2)
            col_aux = make_collector(uavcan.equipment.gnss.Auxiliary)
            col_mag = make_collector(uavcan.equipment.ahrs.MagneticFieldStrength)
            col_pressure = make_collector(uavcan.equipment.air_data.StaticPressure)
            col_temp = make_collector(uavcan.equipment.air_data.StaticTemperature)

            def check_everything():
                check_status()

                try:
                    m = col_pressure[node_id].message
                except KeyError:
                    abort('Pressure measurements are not available. Check the sensor.')
                else:
                    if not 50000 < m.static_pressure < 150000:
                        abort('Invalid pressure reading: %d Pascal. Check the sensor.', m.static_pressure)

                try:
                    m = col_temp[node_id].message
                except KeyError:
                    abort('Temperature measurements are not available. Check the sensor.')
                else:
                    if not 10 < (m.static_temperature - 273.15) < 50:
                        abort('Invalid temperature reading: %d Kelvin. Check the sensor.', m.static_temperature)

                try:
                    m = col_mag[node_id].message
                except KeyError:
                    abort('Magnetic field measurements are not available. Check the sensor.')
                else:
                    magnetic_field_scalar = numpy.linalg.norm(m.magnetic_field_ga)
                    if not 0.01 < magnetic_field_scalar < 2:
                        abort('Invalid magnetic field strength reading: %d Gauss. Check the sensor.',
                              magnetic_field_scalar)

            imperative('Testing GNSS performance.')
            info('Waiting for GNSS fix...')
            with time_limit(GNSS_FIX_TIMEOUT, 'GNSS fix timeout. Check the RF circuit, AFE, antenna, and receiver'):
                while True:
                    safe_spin(1)
                    check_everything()
                    sats_visible = col_aux[node_id].message.sats_visible
                    sats_used = col_aux[node_id].message.sats_used
                    sys.stdout.write('\rsat stats: visible %d, used %d   \r' % (sats_visible, sats_used))
                    sys.stdout.flush()
                    if col_fix[node_id].message.status >= 3:
                        break

            info('Waiting for %d satellites...', GNSS_MIN_SAT_NUM)
            with time_limit(GNSS_MIN_SAT_TIMEOUT,
                            'GNSS performance is degraded. Could be caused by incorrectly assembled RF circuit.'):
                while True:
                    safe_spin(0.5)
                    check_everything()
                    num = col_fix[node_id].message.sats_used
                    cov = list(col_fix[node_id].message.covariance)
                    sys.stdout.write('\r%d sats, covariance: %r      \r' % (num, cov))
                    sys.stdout.flush()
                    if num >= GNSS_MIN_SAT_NUM:
                        break

            check_everything()

            info('Last sampled sensor measurements are provided below. They appear to be correct.')
            info('GNSS fix: %r', col_fix[node_id].message)
            info('GNSS aux: %r', col_aux[node_id].message)
            info('Magnetic field [Ga]: %r', col_mag[node_id].message)
            info('Pressure [Pa]: %r', col_pressure[node_id].message)
            info('Temperature [K]: %r', col_temp[node_id].message)

            # Finalizing the test
            info('Resetting the configuration to factory default...')
            enforce(request(uavcan.protocol.param.ExecuteOpcode.Request(
                opcode=uavcan.protocol.param.ExecuteOpcode.Request().OPCODE_ERASE)).ok,
                'Could not erase configuration')

            request(uavcan.protocol.RestartNode.Request(
                magic_number=uavcan.protocol.RestartNode.Request().MAGIC_NUMBER),
                fire_and_forget=True)

            with BackgroundSpinner(safe_spin, 0.1):
                wait_for_boot()

            wait_for_init()
            check_status()
            log_all_params()
        except Exception:
            for nid in nsmon.get_all_node_id():
                print('Node state: %r' % nsmon.get(nid))
            raise

    # Blocking questions are moved out of the node scope because blocking breaks CAN communications
    # Note that we must instantiate the driver in order to ensure proper traffic LED behavior
    # Periodic calls to receive() are needed to avoid RX queue overflow
    with closing(uavcan.make_driver(iface, bitrate=CAN_BITRATE)) as d:
        with BackgroundSpinner(lambda: d.receive(0.01)):
            if not input('Is the PPS LED blinking once per second?', yes_no=True, default_answer=True):
                abort('PPS LED is not working')

            if not input('Is the CAN1 LED blinking or glowing solid?', yes_no=True, default_answer=True):
                abort('CAN1 LED is not working (however the interface is fine)')

            if not input('Is the STATUS LED blinking once per second?', yes_no=True, default_answer=True):
                abort('STATUS LED is not working')

            # Testing CAN2
            input('1. Disconnect CAN1 and connect to CAN2\n'
                  '2. Terminate CAN2\n'
                  '3. Press ENTER')

            if not input('Is the CAN2 LED blinking or glowing solid?', yes_no=True, default_answer=True):
                abort('Either CAN2 or its LED are not working')


def init_can_iface():
    if '/' not in args.iface:
        logger.debug('Using iface %r as SocketCAN', args.iface)
        execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                              args.iface, args.iface, CAN_BITRATE)
        return args.iface
    else:
        logger.debug('Using iface %r as SLCAN', args.iface)

        # We don't want the SLCAN daemon to interfere...
        execute_shell_command('killall -INT slcand &> /dev/null', ignore_failure=True)
        time.sleep(1)

        # Making sure the interface can be open
        with open(args.iface, 'bw') as _f:
            pass

        return args.iface


def check_interfaces():
    ok = True

    def test_serial_port(glob, name):
        try:
            with open_serial_port(glob):
                info('%s port is OK', name)
                return True
        except Exception:
            error('%s port is not working', name)
            return False

    info('Checking interfaces...')
    ok = test_serial_port(DEBUGGER_PORT_GDB_GLOB, 'GDB') and ok
    ok = test_serial_port(DEBUGGER_PORT_CLI_GLOB, 'CLI') and ok
    try:
        init_can_iface()
        info('CAN interface is OK')
    except Exception:
        logging.debug('CAN check error', exc_info=True)
        error('CAN interface is not working')
        ok = False

    if not ok:
        fatal('Required interfaces are not available. Please check your hardware configuration. '
              'If this application is running on a virtual machine, make sure that hardware '
              'sharing is configured correctly.')

check_interfaces()

licensing_api = make_api_context_with_user_provided_credentials()

with CLIWaitCursor():
    print('Please wait...')
    firmware_data = get_firmware()


def process_one_device(set_device_info):
    out = input('1. Connect DroneCode Probe to the debug connector\n'
                '2. Connect CAN to the first CAN1 connector on the device; terminate the other CAN1 connector\n'
                '3. Connect USB to the device, and make sure that no other Zubax GNSS is connected\n'
                '4. If you want to skip firmware upload, type F\n'
                '5. Press ENTER')

    skip_fw_upload = 'f' in out.lower()
    if not skip_fw_upload:
        info('Loading the firmware')
        with CLIWaitCursor():
            load_firmware(firmware_data)
        info('Waiting for the device to boot...')
        wait_for_boot()
    else:
        info('Firmware upload skipped')

    info('Identifying the connected device...')
    with open_serial_port(USB_CDC_ACM_GLOB, wait_for_port=5) as io:
        logger.info('USB CLI is on %r', io.port)
        zubax_id = SerialCLI(io, 0.1).read_zubax_id()
        product_id = zubax_id['product_id']
        unique_id = b64decode(zubax_id['hw_unique_id'])
        set_device_info(product_id, unique_id)

    info('Testing UAVCAN interface...')
    test_uavcan()

    info('Connecting via USB...')
    with open_serial_port(USB_CDC_ACM_GLOB, wait_for_port=5) as io:
        logger.info('USB CLI is on %r', io.port)
        cli = SerialCLI(io, 0.1)

        enforce(unique_id == b64decode(cli.read_zubax_id()['hw_unique_id']), 'UID changed!')

        try:
            # Using first command to get rid of any garbage lingering in the buffers
            cli.write_line_and_read_output_lines_until_timeout('systime')
        except Exception:
            pass

        # Getting the signature
        info('Requesting signature for unique ID %s', binascii.hexlify(unique_id).decode())
        gensign_response = licensing_api.generate_signature(unique_id, PRODUCT_NAME)
        if gensign_response.new:
            info('New signature has been generated')
        else:
            info('This particular device has been signed earlier, reusing existing signature')
        base64_signature = b64encode(gensign_response.signature).decode()
        logger.info('Generated signature in Base64: %s', base64_signature)

        # Installing the signature; this may fail if the device has been signed earlier - the failure will be ignored
        out = cli.write_line_and_read_output_lines_until_timeout('signature %s', base64_signature)
        logger.debug('Signature installation response (may fail, which is OK): %r', out)

        # Reading the signature back and verifying it
        out = cli.write_line_and_read_output_lines_until_timeout('signature')
        enforce(len(out) == 1, 'Could not read the signature back. Returned lines: %r', out)
        logger.info('Installed signature in Base64: %s', out[0])
        enforce(b64decode(out[0]) == gensign_response.signature,
                'Written signature does not match the generated signature')

        info('Signature has been installed and verified')

run(licensing_api, process_one_device)
