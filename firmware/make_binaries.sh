#!/bin/bash

BOOTLOADER_SIZE_KB=16

# TODO: teach the make descriptor script format the file name automatically
SIGNED_BINARY="com.zubax.gnss-2.0-3.0.`git rev-parse --short=8 HEAD`.uavcan.bin"

./make_can_boot_descriptor.py -v build/zubax_gnss.bin build/$SIGNED_BINARY

dd if=/dev/zero ibs=1k count=$BOOTLOADER_SIZE_KB | tr "\000" "\377" >build/padded_bootloader.bin
dd if=bootloader.bin of=build/padded_bootloader.bin conv=notrunc

cat build/padded_bootloader.bin build/$SIGNED_BINARY > build/signed_with_bootloader.bin

#
# Generating the combined ELF
# Based on https://gist.github.com/tangrs/4030336
#
ld=`tempfile`
tmp=`tempfile`
cat > $ld <<EOF
SECTIONS
{
EOF
echo " . = 0x08000000;" >> $ld
cat >> $ld <<EOF
  .text : { *(.text) }
}
EOF

CROSS_PREFIX=arm-none-eabi-

${CROSS_PREFIX}ld -b binary -r -o $tmp build/signed_with_bootloader.bin
${CROSS_PREFIX}objcopy --rename-section .data=.text --set-section-flags .data=alloc,code,load $tmp
${CROSS_PREFIX}ld $tmp -T $ld -o build/signed_with_bootloader.elf
${CROSS_PREFIX}strip -s build/signed_with_bootloader.elf

rm -f $ld $tmp
