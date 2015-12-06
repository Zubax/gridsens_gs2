#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "You are not root. Why $USER, why?!" 1>&2
    exit 1
fi

set -o xtrace

if ! which arm-none-eabi-gdb; then
    apt-get install -y gcc-arm-none-eabi gdb-arm-none-eabi
fi

apt-get install -y python3 python3-pip

pip3 install colorama easywebdav pyserial numpy pyyaml
