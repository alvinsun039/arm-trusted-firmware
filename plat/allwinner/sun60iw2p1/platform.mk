#
# Copyright (c) 2026, A733 open-source BL31 contributors
#
# SPDX-License-Identifier: BSD-3-Clause
#

AW_PLAT			:=	plat/allwinner
SUN60IW2P1_PLAT		:=	${AW_PLAT}/sun60iw2p1

PLAT_INCLUDES		:=	-I${SUN60IW2P1_PLAT}/include		\
				-Iinclude/plat/arm/common/aarch64	\
				-Idrivers/arm/gic/v3

# NOTE: build with SPD=opteed on the make command line (see build.sh).
# The stock firmware ships optee_sun60iw2p1.bin and the vendor BSP
# registers the opteed_std/opteed_fast runtime services; SPD is consumed
# by the top-level Makefile before platform.mk is included, so it cannot
# be set here.

# BL31 runs from DRAM at 0x48000000 (boot0 monitor container)
SUNXI_BL31_IN_DRAM	:=	1
SUNXI_PSCI_USE_SCPI	:=	0
SUNXI_PSCI_USE_NATIVE	:=	1

# The boot package may leave BL32 out (a BL33-only image is valid on this
# board).  With the option set, opteed skips its init instead of asserting.
SUNXI_OPTEE_OPTIONAL	?=	1
$(eval $(call add_define,SUNXI_OPTEE_OPTIONAL))

# Leave system-suspend stage markers in the RTC general-purpose registers
# (the console UART is unusable in the resume window).  Off by default;
# enable with SUNXI_S3_TRACE=1 for bring-up.
SUNXI_S3_TRACE		?=	0
$(eval $(call add_define,SUNXI_S3_TRACE))

include lib/xlat_tables_v2/xlat_tables.mk
include lib/libfdt/libfdt.mk

# A733 ships a GIC-600
GICV3_IMPL		:=	GIC600
GICV3_SUPPORT_GIC600	:=	1
include drivers/arm/gic/v3/gicv3.mk

PLAT_BL_COMMON_SOURCES	:=	${XLAT_TABLES_LIB_SRCS}			\
				drivers/ti/uart/aarch64/16550_console.S	\
				plat/common/aarch64/crash_console_helpers.S \
				${SUN60IW2P1_PLAT}/plat_helpers.S	\
				${SUN60IW2P1_PLAT}/sunxi_common.c

BL31_SOURCES		+=	${GICV3_SOURCES}			\
				plat/common/plat_gicv3.c		\
				plat/common/plat_psci_common.c		\
				common/desc_image_load.c		\
				drivers/delay_timer/delay_timer.c	\
				drivers/delay_timer/generic_delay_timer.c \
				common/fdt_wrappers.c			\
				lib/cpus/aarch64/cortex_a76.S		\
				lib/cpus/aarch64/cortex_a55.S		\
				${SUN60IW2P1_PLAT}/sunxi_bl31_setup.c	\
				${SUN60IW2P1_PLAT}/sunxi_topology.c	\
				${SUN60IW2P1_PLAT}/sunxi_cpu_ops.c	\
				${SUN60IW2P1_PLAT}/sunxi_pm.c		\
				${SUN60IW2P1_PLAT}/sunxi_arisc.c	\
				${SUN60IW2P1_PLAT}/sunxi_efuse.c		\
				${SUN60IW2P1_PLAT}/sunxi_drm.c		\
				${SUN60IW2P1_PLAT}/sunxi_sip_svc.c

# Single cluster boot; secondary cores enter through the PSCI path.
COLD_BOOT_SINGLE_CPU	:=	1
RESET_TO_BL31		:=	1
# boot0 jumps straight into BL31 at a fixed address; no warm-boot mailbox
PROGRAMMABLE_RESET_ADDRESS	:=	1
HW_ASSISTED_COHERENCY	:=	1
USE_COHERENT_MEM	:=	0
CTX_INCLUDE_AARCH32_REGS:=	0
SEPARATE_CODE_AND_RODATA:=	1
# A55/A76 implement neither SVE nor SPE.  ENABLE_SVE_FOR_NS=1 makes
# cm_manage_extensions_el3() execute `msr zcr_el3`, which is an undefined
# instruction on these cores; the vendor BSP never touches ZCR_EL3.
ENABLE_SVE_FOR_NS	:=	0
ENABLE_SPE_FOR_LOWER_ELS:=	0

# The NS payload (Allwinner BSP 32-bit U-Boot) runs at AArch32 EL1 under a
# monitor whose EL2-unused init must match the vendor BSP (0x4944):
# the default INIT_UNUSED_NS_EL2=0 skips programming HCR_EL2/CPTR_EL2/...,
# leaving boot0 leftovers.  (The A32 SMC dispatch is un-gated in
# common/runtime_svc.c because the A76 forces CTX_INCLUDE_AARCH32_REGS=0.)
INIT_UNUSED_NS_EL2	:= 1

# Errata applied by the vendor monitor: its errata table lists
# dsu_798953, dsu_936184, the A55 set
# (768277 778703 798797 846532 903758 1221012 1530923) and the A76 set
# below.  This boot chain has no BL1 and TF-A's generic code never runs
# the cpu_ops reset functions in a BL31-only build, so
# sunxi_apply_cpu_errata() calls them on every core (cold and warm).
# Without the DSU workarounds (which disable DSU clock gating) a
# freshly powered-on core wedges the interconnect on its first
# shareable access - the whole system dies silently.
ERRATA_A55_768277		:= 1
ERRATA_A55_778703		:= 1
ERRATA_A55_798797		:= 1
ERRATA_A55_846532		:= 1
ERRATA_A55_903758		:= 1
# The A55's erratum-1221012 block is the only writer of the
# CPUPOR/CPUPMR/CPUPCR/CPUPSELR "power-control overrides" that disturb
# the secondary PPU power-up; the vendor monitor contains no
# writes to those registers anywhere, so this block stays disabled
# while every other workaround - including the DSU clock-gating fixes
# (798953/936184) - is applied at both cold boot and warm boot.
ERRATA_A55_1221012		:= 0
ERRATA_A55_1530923		:= 1
ERRATA_DSU_798953		:= 1
ERRATA_DSU_936184		:= 1
ERRATA_A76_1073348		:= 1
ERRATA_A76_1130799		:= 1
ERRATA_A76_1165522		:= 1
ERRATA_A76_1220197		:= 1
ERRATA_A76_1257314		:= 1
ERRATA_A76_1262606		:= 1
ERRATA_A76_1262888		:= 1
ERRATA_A76_1275112		:= 1
ERRATA_A76_1286807		:= 1
ERRATA_A76_1791580		:= 1
ERRATA_A76_1868343		:= 1
ERRATA_A76_1946160		:= 1
