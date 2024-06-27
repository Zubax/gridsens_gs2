#!/bin/bash

set -o xtrace

if ! which arm-none-eabi-gdb; then
    echo "The toolchain does not seem to be installed correctly; please read this: https://kb.zubax.com/x/NoEh"
fi

sudo apt-get install -y python3.8 python3.8-distutils can-utils || exit 1

python3.8 -m pip install colorama easywebdav pyserial numpy pyyaml || exit 2
