Zubax GNSS
==========

Zubax GNSS 2 is a multipurpose high-performance positioning module interfaced via CAN bus, USB, and UART.
It includes a state-of-the-art multi-system GPS/GLONASS receiver, a high-precision barometric altimeter,
and a 3-axis compass with thermal compensation.
Zubax GNSS 2 supports variety of standard protocols, which ensures compatibility with most of existing
software and hardware: [UAVCAN](/uavcan) (over CAN bus), NMEA 0183 (over USB and UART),
and u-Blox M8 protocol.

* [**LEARN MORE**](http://zubax.com/product/zubax-gnss-2)
* [**DOCUMENTATION AND TUTORIALS**](http://docs.zubax.com/zubax_gnss_2)

## Revisions

The following table documents existing modifications of Zubax GNSS hardware and compatible firmware versions.

Hardware                        | Compatible firmware versions
--------------------------------|------------------------------
Zubax GNSS v1 (year 2014)       | 1.0, 2.0
Zubax GNSS v2 (year 2015)       | 3.x

Each firmware version has a dedicated branch named `release_X.X`, where `X.X` stands for the version number.
Please check out the corresponding branch to see instructions specific for your firmware version.
Master branch always contains the newest version of firmware and it may be unstable.

## Getting the bootloader

The bootloader image is conveniently included in this repository as `bootloader.bin`.
It can be built using the target `zubaxgnss-v1_bootloader` of <https://github.com/Zubax/PX4Firmware>
(branch `nuttx_next`).

In future the bootloader will be extracted into a separate project.

The bootloader can be flashed either independently, or as a combined image together with the application (see below).

## Building the firmware

* Install ARM GCC toolchain version 4.9 or newer
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

* `com.zubax.*.uavcan.bin` - application binary suitable for UAVCAN bootloading, with correct image CRC.
* `com.zubax.*.compound.bin` - above image combined with the bootloader; can be flashed on an empty MCU.
* `compound.elf` - ELF file with embedded bootloader and correct image CRC; can be used for symbol
debugging. Since this ELF includes the bootloader and has correct image CRC, it can be flashed and executed directly
with an SWD debugger, no extra steps required.

## Loading the firmware

### Using in-circuit debuggers

Avialable loaders:

```shell
cd ./zubax_chibios/tools

./blackmagic_flash.sh   # Black Magic Debug Probe
./stlink_flash.sh       # ST-Link v2
```

### Using UAVCAN bootloader

Feed the signed binary (it will be named as `*.uavcan.bin`) to the UAVCAN firmware update server.
For example, see these instructions for PX4: <http://dev.px4.io/uavcan-node-firmware.html>.

Source of the UAVCAN bootloader for this board can be found at the
[PX4 source repository](https://github.com/PX4/Firmware).

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
