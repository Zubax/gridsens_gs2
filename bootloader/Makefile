#
# Copyright (C) 2014-2017  Zubax Robotics  <info@zubax.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

PROJECT = com.zubax.gnss

HW_VERSION_MAJOR = 2
BL_VERSION_MAJOR = 1
BL_VERSION_MINOR = 1

APPLICATION_OFFSET = 32768

#
# Bootloader
#

CSRC = $(shell find src -name '*.c')

CPPSRC = $(shell find src -name '*.cpp')

UINCDIR = src                                            \
          src/os_config

UDEFS = -DAPPLICATION_OFFSET=$(APPLICATION_OFFSET)       \
        -DBL_VERSION_MAJOR=$(BL_VERSION_MAJOR)           \
        -DBL_VERSION_MINOR=$(BL_VERSION_MINOR)           \
        -DHW_VERSION_MAJOR=$(HW_VERSION_MAJOR)           \
        -DPRODUCT_ID_STRING=\"$(PROJECT)\"               \
        -DPRODUCT_NAME_STRING=\"Zubax\ GNSS\"

#
# Libcanard
#

UINCDIR += libcanard                                     \
           libcanard/drivers/stm32

CSRC += libcanard/canard.c                               \
        libcanard/drivers/stm32/canard_stm32.c

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# ChibiOS
#

USE_EXCEPTIONS_STACKSIZE := 0x0400
USE_PROCESS_STACKSIZE    := 0x0800

USE_LTO := yes

UDEFS += -DWATCHDOG_RETAIN_RESET_CAUSE_CODE=1            \
         -DOS_USE_CHPRINTF=1                             \
         -DAGGRESSIVE_SIZE_OPTIMIZATION=1                \
         -DCRT1_AREAS_NUMBER=0

SERIAL_CLI_PORT_NUMBER = 3

BUILD_BOOTLOADER = 1

CPPWARN := -Wshadow -Wpointer-arith -Wno-packed -Wno-attributes -Wno-error=undef -Wno-error=shadow

RELEASE_OPT = -Os -fomit-frame-pointer
DEBUG_OPT = -Os -g3 -DDISABLE_WATCHDOG=1

LDSCRIPT = ld.ld

include zubax_chibios/rules_stm32f105_107.mk
