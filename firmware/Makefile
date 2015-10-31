#
# Copyright (c) 2014 Zubax, zubax.com
# Distributed under the MIT License, available in the file LICENSE.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

PROJECT = zubax_gnss

PRODUCT_ID_STRING = com.zubax.gnss
PRODUCT_NAME_STRING = Zubax\ GNSS

HW_VERSION = 2

#
# Application
#

CSRC = $(shell find src -name '*.c')

CPPSRC = $(shell find src -name '*.cpp')

UINCDIR = src                \
          src/sys

UDEFS = -DFW_VERSION_MAJOR=3                             \
        -DFW_VERSION_MINOR=0                             \
        -DHW_VERSION=$(HW_VERSION)                       \
        -DPRODUCT_ID_STRING=\"$(PRODUCT_ID_STRING)\"     \
        -DPRODUCT_NAME_STRING=\"$(PRODUCT_NAME_STRING)\"

#
# UAVCAN library
#

UDEFS += -DUAVCAN_STM32_TIMER_NUMBER=7    \
         -DUAVCAN_STM32_NUM_IFACES=2      \
         -DUAVCAN_STM32_CHIBIOS=1

include libuavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

include libuavcan/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# ChibiOS
#

BOOTLOADER_SIZE = 16384

DDEFS += -DCORTEX_VTOR_INIT=$(BOOTLOADER_SIZE)

SERIAL_CLI_PORT_NUMBER = 3

CPPWARN := -Wundef -Wno-error=undef

RELEASE_OPT = -Os -fomit-frame-pointer
DEBUG_OPT = -Os -g3

LDSCRIPT = ld.ld

binaries: build/zubax_gnss.bin
	cd build && ../make_can_boot_descriptor.py $(PROJECT).bin $(PRODUCT_ID_STRING) $(HW_VERSION).0
	cd build && ../make_can_boot_descriptor.py $(PROJECT).bin $(PRODUCT_ID_STRING) $(HW_VERSION).0 \
	                                           --bootloader-image-size=$(BOOTLOADER_SIZE)          \
	                                           --bootloader-image=../bootloader.bin                \
	                                           --output-prefix='withbootloader-'

include zubax_chibios/rules_stm32f105_107.mk
