# How to flash GNSS using Dr Watson

This tutorial demonstrates how to flash Zubax GNSS 2 using Dr. Watson on a bunker laptop.

## Required parts

The following parts will be needed:

-   1 x Zubax Babel
-   1 x Dronecode Probe v2.3
-   1 x UAVCAN Micro patch cable
-   1 x DCD-M Cable
-   1 x UAVCAN/CAN Micro termination plug
-   3 X Micro USB cable Type B   
     

## Connecting

Connect all hardware to bunker laptop.

![1615029256298.jpeg](/docs/img//1615029256298.jpeg)
-   Connect DroneCode Probe to the debug connector.
-   Connect CAN to the first CAN1 connector on the device; terminate the other CAN1 connector.
-   Connect USB to the device.

![1615032729267.jpeg](/docs/img/1615032729267.jpeg)

## Flashing

Use `*dmesg*` command to find out the port number of Zubax Babel (alternatively, use udev symlinks like `/dev/serial/by-id/*Babel*`).

![screenshot_from_2021-03-06_13-44-27.png](/docs/img/screenshot_from_2021-03-06_13-44-27.png)
From the Dr Watson root directory ( /zubax\_gnss/tools/drwatson) execute.

`sudo ./drwatson_zubax_gnss.py /dev/ttyACM0`

Note: Passed argument should be the serial port number of the attached Zubax Babel
![screenshot_from_2021-03-06_13-59-00.png](/docs/img/screenshot_from_2021-03-06_13-59-00.png)

After that follow the on-screen instructions to complete the flashing process.