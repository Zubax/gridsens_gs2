Courierdrone GPS
================

Under construction.

The documentation provided below is preliminary and likely to be changed in future.

## Building the firmware

* Install ARM GCC toolchain version 4.8 or newer
* Obtain the sources:
```shell
git clone https://github.com/Courierdrone/crdr_gps
cd crdr_gps/
git submodule update --init --recursive
cd ..
git clone https://github.com/pavel-kirienko/uavcan  # Or make a symlink instead
```
* Install the libuavcan dependencies:
```shell
# Python packages for DSDL compiler:
pip install mako
```
* Build:
```shell
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```
* Flash the board using any available flash loader. For instance:
```shell
./crdr_chibios/tools/blackmagic_flash.sh   # Black Magic Debug Probe
./crdr_chibios/tools/stlink_flash.sh       # ST-Link v2
```

## Configuring the board

The board can be configured either via serial CLI over UART or via UAVCAN.

Power can be supplied via either or all CAN connectors, or via the UART connector. Max power requirements: 5VDC 200mA.

### Serial CLI

Connect an UART adapter, power on the board. If the UART was not connected at the time the board was powered, the CLI
may be available in read only mode (safety feature).

UART parameters:

* Baudrate: 115200
* Word width: 8
* Parity: no
* Stop bits: 1

CLI parameters:

* Line ending: LF (0x0A, \n)
* Local echo: off

Basic CLI commands:

* `cfg` - change, save or reset the board configuration
  * `cfg help` - crash course on command usage
  * `cfg list` - list all available configuration parameters and their values
* `gnssbridge` - tunnel the GNSS receiver port through CLI port
* `bootloader` - start the STM32's embedded bootloader; can be used for firmware update

### UAVCAN

Use the standard configuration service `uavcan.protocol.param.*`.

## UAVCAN interface

The board features doubly redundant CAN interface ISO 11898-2:2003. If the board is being connected to a system with single CAN bus, any interface can be used.

Default UAVCAN config:

* CAN bitrate: 1 Mbps
* UAVCAN Node ID: 1

The board requires no mandatory configuration to begin functioning with an UAVCAN network.
