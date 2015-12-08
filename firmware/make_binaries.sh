#!/bin/bash

BOOTLOADER_SIZE_KB=8
SIGNED_BINARY="com.zubax.gnss-1.0-2.0.`git rev-parse --short=8 HEAD`.uavcan.bin"

./make_can_boot_descriptor.py -v build/zubax_gnss.bin build/$SIGNED_BINARY

dd if=/dev/zero ibs=1k count=$BOOTLOADER_SIZE_KB | tr "\000" "\377" >build/padded_bootloader.bin
dd if=bootloader.bin of=build/padded_bootloader.bin conv=notrunc

cat build/padded_bootloader.bin build/$SIGNED_BINARY > build/signed_with_bootloader.bin
