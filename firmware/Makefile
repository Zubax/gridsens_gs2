#
# Copyright (c) 2014 Courierdrone, courierdrone.com
# Distributed under the MIT License, available in the file LICENSE.
# Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
#

PROJECT = crdr_gps

#
# Application
#

CSRC = $(shell find src -name '*.c')

CPPSRC = $(shell find src -name '*.cpp')

UINCDIR = src/sys

UDEFS = -DFW_VERSION_MAJOR=0 -DFW_VERSION_MINOR=1 -DHW_VERSION=0

#
# UAVCAN library
#

UDEFS += -DUAVCAN_STM32_TIMER_NUMBER=7    \
         -DUAVCAN_STM32_CHIBIOS=1         \
         -DUAVCAN_MEM_POOL_BLOCK_SIZE=56  \
         -DUAVCAN_TOSTRING=0              \
         -DUAVCAN_EXCEPTIONS=0

include uavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

include uavcan/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated

#
# ChibiOS
#

SERIAL_CLI_PORT_NUMBER = 1

RELEASE_OPT = -Os -fomit-frame-pointer
DEBUG_OPT = -Og -g3

include crdr_chibios/rules_stm32f105_107.mk
