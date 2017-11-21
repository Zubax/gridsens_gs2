#
# Copyright (C) 2014-2015  Zubax Robotics  <info@zubax.com>
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
FW_VERSION_MAJOR = 4
FW_VERSION_MINOR = 1

#
# Application
#

CSRC = $(shell find src -name '*.c')

CPPSRC = $(shell find src -name '*.cpp')

UINCDIR = src                \
          src/sys

UDEFS = -DFW_VERSION_MAJOR=$(FW_VERSION_MAJOR)           \
        -DFW_VERSION_MINOR=$(FW_VERSION_MINOR)           \
        -DHW_VERSION_MAJOR=$(HW_VERSION_MAJOR)           \
        -DPRODUCT_ID_STRING=\"$(PROJECT)\"               \
        -DPRODUCT_NAME_STRING=\"Zubax\ GNSS\"

#
# UAVCAN library
# On CAN IRQ priority read this: http://forum.chibios.org/phpbb/viewtopic.php?f=25&t=3085
#

UDEFS += -DUAVCAN_STM32_TIMER_NUMBER=7          \
         -DUAVCAN_STM32_NUM_IFACES=2            \
         -DUAVCAN_STM32_CHIBIOS=1               \
         -DUAVCAN_CPP_VERSION=UAVCAN_CPP11      \
         -DUAVCAN_STM32_IRQ_PRIORITY_MASK=4

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

USE_EXCEPTIONS_STACKSIZE := 0x0800
USE_PROCESS_STACKSIZE    := 0x0800

BUILD_CONFIG = 1
BUILD_CHIBIOS_SHELL = 1

BOOTLOADER_SIZE = 32768

DDEFS += -DCORTEX_VTOR_INIT=$(BOOTLOADER_SIZE)            \
         -DCRT1_AREAS_NUMBER=0

SERIAL_CLI_PORT_NUMBER = 3

CPPWARN := -Wundef -Wno-error=undef

RELEASE_OPT = -O2 -fomit-frame-pointer
DEBUG_OPT = -Os -g3 -DDISABLE_WATCHDOG=1

LDSCRIPT = ld.ld

# Note that we're explicitly specifying the required minor hardware version here.
# TODO WARNING FIXME This versioning trick is actually a hack. In the future we should AVOID releasing
#                    firmware-incompatible hardware versions that share the same major version number.
#                    In other words, all firmware images that share the same major number should be compatible
#                    with all versions of hardware that share the same major version number. Ideally the major
#                    version numbers for the hardware and the software should match.
HW_VERSION_MAJOR_MINOR := $(HW_VERSION_MAJOR).2
FW_VERSION_MAJOR_MINOR_VCS_HASH := $(FW_VERSION_MAJOR).$(FW_VERSION_MINOR).$(GIT_HASH)
COMPOUND_IMAGE_FILE := $(PROJECT)-$(HW_VERSION_MAJOR_MINOR)-$(FW_VERSION_MAJOR_MINOR_VCS_HASH).compound.bin

BOOTLOADER_DIR := $(abspath ../bootloader)
BOOTLOADER_IMAGE := $(BOOTLOADER_DIR)/build/com.zubax.gnss.bin

.PHONY: binaries
binaries: build/$(PROJECT).bin build/$(PROJECT).elf $(BOOTLOADER_IMAGE)
	# Removing previous build outputs that could use a different git hash
	rm -rf build/*.application.bin build/*.compound.bin
	
	# Generating compound image with embedded bootloader
	cd build && dd if=/dev/zero bs=$(BOOTLOADER_SIZE) count=1 | tr "\000" "\377" >padded_bootloader.tmp.bin
	cd build && dd if=$(BOOTLOADER_IMAGE) of=padded_bootloader.tmp.bin conv=notrunc
	cd build && cat padded_bootloader.tmp.bin $(PROJECT).bin >$(COMPOUND_IMAGE_FILE)
	
	# Generating the signed image for the bootloader
	cd build && ../zubax_chibios/tools/make_boot_descriptor.py $(PROJECT).bin $(PROJECT) $(HW_VERSION_MAJOR_MINOR) \
	                                                           --also-patch-descriptor-in=$(PROJECT).elf           \
	                                                           --also-patch-descriptor-in=$(COMPOUND_IMAGE_FILE)
	
	# Injecting the bootloader into the final ELF
	cd build && $(TOOLCHAIN_PREFIX)-objcopy --add-section bootloader=$(BOOTLOADER_IMAGE)   \
	                                        --set-section-flags bootloader=load,alloc      \
	                                        --change-section-address bootloader=0x08000000 \
	                                        $(PROJECT).elf compound.elf
	
	# Removing temporary files
	cd build && rm -f $(PROJECT).bin $(PROJECT).elf *.hex *.tmp.bin

$(BOOTLOADER_IMAGE):
	# Building the bootloader; enforcing RELEASE because it won't fit otherwise
	+cd $(BOOTLOADER_DIR) && make RELEASE=1

.PHONY: upload
upload: build/compound.elf
	./zubax_chibios/tools/blackmagic_flash.sh

include zubax_chibios/rules_stm32f105_107.mk
