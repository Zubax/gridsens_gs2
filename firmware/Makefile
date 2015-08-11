#
# Copyright (c) 2014 Zubax, zubax.com
# Distributed under the MIT License, available in the file LICENSE.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

PROJECT = zubax_gnss

#
# Application
#

CSRC = $(shell find src -name '*.c')

CPPSRC = $(shell find src -name '*.cpp')

UINCDIR = src/sys

UDEFS = -DFW_VERSION_MAJOR=1 -DFW_VERSION_MINOR=0 -DHW_VERSION=1

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

SERIAL_CLI_PORT_NUMBER = 1

CPPWARN := -Wundef -Wno-error=undef

RELEASE_OPT = -Os -fomit-frame-pointer
DEBUG_OPT = -Os -g3

LDSCRIPT = ld.ld

include zubax_chibios/rules_stm32f105_107.mk
