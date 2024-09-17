#
# Copyright (c) 2013-2021, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

include lib/xlat_tables_v2/xlat_tables.mk

include drivers/arm/gic/v2/gicv2.mk

# Inherit the BOOT UART configuration from the VPU
PL011_INHERIT_SETUP     := 1

PLAT_INCLUDES		:=	-Iplat/rpi/common/include		\
				-Iplat/rpi/rpi5/include

PLAT_BL_COMMON_SOURCES	:=	drivers/ti/uart/aarch64/16550_console.S	\
				drivers/arm/pl011/aarch64/pl011_console.S \
				plat/rpi/rpi5/rpi5_private.c		\
				${XLAT_TABLES_LIB_SRCS}

BL31_SOURCES		+=	lib/cpus/aarch64/cortex_a76.S		\
				plat/rpi/rpi5/aarch64/plat_helpers.S	\
				plat/rpi/rpi5/aarch64/armstub8_header.S	\
				drivers/delay_timer/delay_timer.c	\
				drivers/gpio/gpio.c			\
				plat/common/plat_gicv2.c                \
				plat/rpi/rpi5/rpi5_bl31_setup.c		\
				plat/rpi/rpi5/rpi5_topology.c		\
				plat/rpi/rpi5/rpi5_pm.c		\
				plat/common/plat_psci_common.c		\
				${GICV2_SOURCES}
#				drivers/rpi3/gpio/rpi3_gpio.c		\

# For now we only support BL31, using the kernel loaded by the GPU firmware.
RESET_TO_BL31		:=	1

# All CPUs enter armstub8.bin.
COLD_BOOT_SINGLE_CPU	:=	0

# Tune compiler for Cortex-A76
ifeq ($(notdir $(CC)),armclang)
    TF_CFLAGS_aarch64	+=	-mcpu=cortex-a76
else ifneq ($(findstring clang,$(notdir $(CC))),)
    TF_CFLAGS_aarch64	+=	-mcpu=cortex-a76
else
    TF_CFLAGS_aarch64	+=	-mtune=cortex-a76
endif

# Add support for platform supplied linker script for BL31 build
$(eval $(call add_define,PLAT_EXTRA_LD_SCRIPT))

# Enable all errata workarounds for Cortex-A76
ERRATA_A76_1257314 := 0
ERRATA_A76_1262606 := 0
ERRATA_A76_1262888 := 0
ERRATA_A76_1275112 := 0
ERRATA_A76_1286807 := 1
ERRATA_A76_1791580 := 0
ERRATA_A76_1165522 := 1
ERRATA_A76_1868343 := 0
ERRATA_A76_1946160 := 1

# Add new default target when compiling this platform
all: bl31

# Build config flags
# ------------------

# Disable stack protector by default
ENABLE_STACK_PROTECTOR	 	:= 0

# Have different sections for code and rodata
SEPARATE_CODE_AND_RODATA	:= 1

# System coherency is managed in hardware
HW_ASSISTED_COHERENCY   :=      1

# When building for systems with hardware-assisted coherency, there's no need to
# use USE_COHERENT_MEM. Require that USE_COHERENT_MEM must be set to 0 too.
USE_COHERENT_MEM        :=      0

CTX_INCLUDE_AARCH32_REGS := 0

# Platform build flags
# --------------------

# There is not much else than a Linux kernel to load at the moment.
RPI5_DIRECT_LINUX_BOOT		:= 1

# UART to use at runtime. -1 means the runtime UART is disabled.
# Any other value means the default UART will be used.
RPI3_RUNTIME_UART		:= 0
RPI5_RUNTIME_UART		:= 0

# Use normal memory mapping for ROM, FIP, SRAM and DRAM
RPI5_USE_UEFI_MAP		:= 0

# SMCCC PCI support (should be enabled for ACPI builds)
SMC_PCI_SUPPORT            	:= 0

# Process platform flags
# ----------------------

$(eval $(call add_define,RPI5_DIRECT_LINUX_BOOT))
ifdef RPI5_PRELOADED_DTB_BASE
$(eval $(call add_define,RPI5_PRELOADED_DTB_BASE))
endif
# $(eval $(call add_define,RPI3_RUNTIME_UART))
$(eval $(call add_define,RPI5_RUNTIME_UART))
$(eval $(call add_define,RPI5_USE_UEFI_MAP))
$(eval $(call add_define,SMC_PCI_SUPPORT))

ifeq (${ARCH},aarch32)
  $(error Error: AArch32 not supported on rpi5)
endif

ifneq ($(ENABLE_STACK_PROTECTOR), 0)
PLAT_BL_COMMON_SOURCES	+=	drivers/rpi3/rng/rpi3_rng.c		\
				plat/rpi/common/rpi3_stack_protector.c
endif

ifeq ($(SMC_PCI_SUPPORT), 1)
BL31_SOURCES            +=      plat/rpi/rpi5/rpi5_pci_svc.c
endif
