Zubax GNSS
==========

Zubax GNSS is a high performance open source positioning module for outdoor environments, interfaced via doubly redundant [UAVCAN bus](http://uavcan.org). It includes a state of the art GPS/GLONASS receiver, a high precision barometer and a 3-axis compass.

* [LEARN MORE](http://zubax.com/product/zubax-gnss)
* [DOCUMENTATION AND TUTORIALS](http://docs.zubax.com/Zubax_GNSS)

## Building the Firmware

* Install ARM GCC toolchain version 4.8 or newer
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
./make_release_binaries.sh
```

## Flashing

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
