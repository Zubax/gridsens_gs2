# Production testing software

This directory contains production testing application for Zubax GNSS based on Drwatson.
Drwatson is a software framework written in Python for hardware production testing automation.

## Installation

This application requires an Ubuntu-based Linux distribution.

After checking out this repository and all of its submodules (see the main README for details),
execute `sudo ./setup.sh`, and you're ready to get started:

```bash
./run.sh --help
```

`run.sh` is just a convenient wrapper that pulls the latest version from git before running Drwatson as superuser.
You can either use it or run drwatson manually as `sudo ./drwatson_zubax_gnss.py`.

## Other documentation

Refer to <https://kb.zubax.com/> to find more documentation about anything.
