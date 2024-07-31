# Production testing software

This directory contains production testing application for Zubax GNSS based on DrWatson.
DrWatson is a software framework written in Python for hardware production testing automation.

## Installation

This application requires an Ubuntu-based GNU/Linux distribution.

A working GNU ARM toolchain version 6 or newer must be installed.
Please read this for installation instructions: <https://kb.zubax.com/x/NoEh>.

The OS must be configured to allow access to virtual serial ports to regular users without superuser privileges.
Read this tutorial for setup instructions: <https://kb.zubax.com/x/N4Ah>.

After checking out this repository and all of its submodules (see the main README for details),
execute `./setup.sh`, and you're ready to get started:

```bash
./run.sh --help
```

`run.sh` is just a convenient wrapper that pulls the latest version from git before running Drwatson.
You can either use it or run drwatson manually as `./drwatson_zubax_gnss.py`.

## Usage

The following parts will be needed:

- Zubax Babel, aka CF1
- Zubax Dronecode Probe, aka BF1
- DCD-M Cable
- UAVCAN Micro patch cable
- UAVCAN/CAN Micro termination plug
- x3 Micro USB cable Type B

Connect all hardware as follows:

- Connect DroneCode Probe to the debug connector.
- Connect CAN to the first CAN1 connector on the device; terminate the other CAN1 connector.
- Connect USB to the device.

From this directory execute:

```
sudo ./drwatson_zubax_gnss.py /dev/serial/by-id/usb*Zubax*Babel*
```

After that follow the on-screen instructions.
