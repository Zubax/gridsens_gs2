Zubax GNSS
==========

Zubax GNSS is a high performance open source positioning module for outdoor environments,
interfaced via doubly redundant [UAVCAN bus](http://uavcan.org) and USB.
It includes a state of the art GPS/GLONASS receiver, a high precision barometer and a thermocompensated 3-axis compass.

* [LEARN MORE](http://zubax.com/product/zubax-gnss)
* [DOCUMENTATION AND TUTORIALS](http://docs.zubax.com/Zubax_GNSS)

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
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```
* Optionally, generate release binaries:
```shell
cd firmware
./make_binaries.sh
```

## Loading the firmware

TODO: explain how to flash the bootloader.

### Using in-circuit debuggers

Avialable loaders:

```shell
./zubax_chibios/tools/blackmagic_flash.sh   # Black Magic Debug Probe
./zubax_chibios/tools/stlink_flash.sh       # ST-Link v2
```

### Using UAVCAN bootloader

First, make sure to generate release binaries as described above.
Then feed the signed binary (it will be named as `*.uavcan.bin`) to the UAVCAN firmware update server.

Source of the UAVCAN bootloader for this board can be found at the
[PX4 source repository](https://github.com/PX4/Firmware).
