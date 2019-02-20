#!/bin/bash

set -o xtrace

if ! which arm-none-eabi-gdb; then
    echo "The toolchain does not seem to be installed correctly; please read this: https://kb.zubax.com/x/NoEh"
fi

sudo apt-get install -y python3-pip can-utils

sudo pip3 install colorama easywebdav pyserial numpy pyyaml
