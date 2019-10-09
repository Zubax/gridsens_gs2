Zubax GNSS
==========

[![Join the chat at https://gitter.im/Zubax/general](https://badges.gitter.im/Zubax/zubax_gnss.svg)](https://gitter.im/Zubax/general?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Zubax GNSS 2 is a multipurpose high-performance positioning module interfaced via CAN bus, USB, and UART.
It includes a state-of-the-art multi-system concurrent GPS+GLONASS+Galileo receiver,
a high-precision barometric altimeter, and a 3-axis compass with thermal compensation.
Zubax GNSS 2 supports a variety of standard protocols, which ensure compatibility with most of existing
software and hardware: [UAVCAN](http://uavcan.org) (over CAN bus), NMEA 0183 (over USB and UART),
and u-Blox M8 protocol.

[**ZUBAX GNSS HOMEPAGE**](https://zubax.com/products/gnss_2)

## Revisions

The following table documents existing modifications of Zubax GNSS hardware and compatible firmware versions.

Hardware                        | Compatible firmware versions  | Branch
--------------------------------|-------------------------------|----------------------------
Zubax GNSS v1 (year 2014)       | 1.x, 2.x                      | `release_1.0`, `release_2.0`
Zubax GNSS v2.1 (year 2015)     | 3.x                           | `release_3.0`
Zubax GNSS v2.2 (year 2017)     | 4.x                           | `master`
Zubax GNSS v2.3 (year 2019)     | 4.x                           | see v2.2

Please check out the corresponding branch to see instructions specific for your firmware version.
Master branch always contains the newest version of firmware and it may be unstable.

## Release notes

Newest entries at the top.

### Zubax GNSS v2.2, firmware v4.1

* Added support for Galileo, since the system is now operational.
Now, Zubax GNSS 2 utilizes GPS+GLONASS+Galileo concurrently.
* Added a configuration parameter that allows the user to change the dynamic model of the vehicle.
Three options are available: Automotive, Sea, Airborne. Airborne is chosen by default.
* Bootloader updated to version 1.1, where the field naming issue in `zubax_id` was resolved.
* Static UAVCAN node ID setting takes precedence over the value provided by the bootloader.

### Zubax GNSS v2.2, firmware v4.0

* New GNSS RF front-end enables even better noise rejection and sensitivity.
* Improved power supply noise filtering.
* New compass: LIS3MDL instead of HMC5983.
* Using Zubax Embedded Bootloader instead of the old PX4 UAVCAN bootloader.
The new bootloader supports USB, UART, and CAN interfaces for firmware update purposes.
Supported protocols are UAVCAN over CAN, and YMODEM/XMODEM/XMODEM-1K over USB and UART.
* Faster sensor update rates.
* Static temperature publishing rate is fixed at 1/5th of the static pressure publishing rate.

### Zubax GNSS v2.1, firmware v3.2

* Increased the default and the maximum compass publishing rate to 100 Hz.
* Magnetometer output is processed through a three sample long median filter in order to work-around a bug in the
HMC5983 sensor which causes it to periodically provide single erroneous measurements.

### Zubax GNSS v2.1, firmware v3.1

* Added an optional magnetic field rescaling feature in order to work around improper scale handling in some autopilots.

## Bootloader

The Zubax Embedded Bootloader allows the end user to upgrade the firmware via
USB (YMODEM/XMODEM/XMODEM-1K), UART (YMODEM/XMODEM/XMODEM-1K), or CAN (UAVCAN).
The sources of the bootloader are located in the dedicated directory.
No special steps are needed in order to build it - the makefile will build it automatically as needed.
Please refer to the official documentation in order to learn more about the bootloader and how to use it.

The bootloader can be flashed either independently, or as a combined image together with the application (see below).

## Building the firmware

If you're not running Linux or OSX natively, you can use
[Bistromathic - a Linux virtual machine pre-configured for embedded development](https://kb.zubax.com/x/KIEh).

* Install the ARM GCC toolchain version 7.3.1.
* Init the sources:
```shell
git submodule update --init --recursive
```
* Build:
```shell
cd firmware
make binaries RELEASE=1 # RELEASE is optional; omit to build the debug version
```

The steps above will produce the following outputs in the build output directory:

* `com.zubax.*.application.bin` - application binary suitable for bootloading, with the correct application descriptor.
* `com.zubax.*.compound.bin` - above image combined with the bootloader; can be flashed on an empty MCU.
* `compound.elf` - ELF file with embedded bootloader and correct application descriptor; can be used for symbol
debugging. Since this ELF includes the bootloader and has correct application descriptor,
it can be flashed and executed directly with an SWD debugger, no extra steps required.

## Loading the firmware

### Using in-circuit debuggers

Avialable loaders:

```shell
cd ./zubax_chibios/tools

./blackmagic_flash.sh   # Black Magic Debug Probe
./stlink_flash.sh       # ST-Link v2
```

### Using the bootloader

Please refer to the official documentation.

## License

The firmware is licensed under the terms of GNU GPL v3.

> Copyright (C) 2015 Zubax Robotics info@zubax.com
>
> This program is free software: you can redistribute it and/or modify it under the terms of the
> GNU General Public License as published by the Free Software Foundation, either version 3 of the License,
> or (at your option) any later version.
>
> This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
> without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
> See the GNU General Public License for more details.
>
> You should have received a copy of the GNU General Public License along with this program.
> If not, see http://www.gnu.org/licenses/.
