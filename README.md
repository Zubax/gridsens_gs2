Zubax GNSS
==========

Zubax GNSS is a high performance open source positioning module for outdoor environments, interfaced via doubly redundant [UAVCAN bus](http://uavcan.org). It includes a state of the art GPS/GLONASS receiver, a high precision barometer and a 3-axis compass.

* [LEARN MORE](http://zubax.com/product/zubax-gnss)
* [DOCUMENTATION AND TUTORIALS](http://docs.zubax.com/Zubax_GNSS)

## Building the Firmware

* Install ARM GCC toolchain version 4.8
* Init the sources:
```shell
git submodule update --init --recursive
```
* Build:
```shell
cd firmware
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```
* Flash the board using any available flash loader. For instance:
```shell
./zubax_chibios/tools/blackmagic_flash.sh   # Black Magic Debug Probe
./zubax_chibios/tools/stlink_flash.sh       # ST-Link v2
```
