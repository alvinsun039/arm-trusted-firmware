/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLATFORM_DEF_H
#define PLATFORM_DEF_H

#include <lib/utils_def.h>
#include <plat/common/common_def.h>

#include <sunxi_mmap.h>

#define PLAT_ALLWINNER_SUN60IW2P1

/* BL31: boot0 loads the monitor container at 0x48000000. The first 4KiB
 * page carries the Allwinner "monitor" header (its `b` jumps into the
 * image); the linked image starts one page later so the runtime base
 * matches the link base without any relocation. */
#define SUNXI_MONITOR_HEADER_SIZE	0x1000
#define BL31_BASE			(SUNXI_BL31_BASE + SUNXI_MONITOR_HEADER_SIZE)
#define BL31_LIMIT			(SUNXI_BL31_BASE + SUNXI_BL31_SIZE)

/* A733: single DynamIQ cluster, 4x Cortex-A76 + 4x Cortex-A55.
 * Allwinner maps the logical core number to MPIDR.Aff1. */
#define PLATFORM_CLUSTER_COUNT		U(1)
#define PLATFORM_MAX_CPUS_PER_CLUSTER	U(8)
#define PLATFORM_CORE_COUNT		U(8)
/* 8 KiB per core: BL31 runs the USB/FES server and its IRQ path at EL3 */
#define PLATFORM_STACK_SIZE		UL(0x2000)

#define PLAT_MAX_PWR_LVL		U(2)
#define PLAT_NUM_PWR_DOMAINS		(U(1) + PLATFORM_CLUSTER_COUNT + \
					 PLATFORM_CORE_COUNT)

#define PLAT_MAX_PWR_LVL_STATES		U(3)
#define PLAT_MAX_RET_STATE		U(1)
#define PLAT_MAX_OFF_STATE		U(2)

/* address space */
/*
 * The vendor BSP's xlat context
 * carries pa_max/va_max = 0x7FFFFFFFF (32 GiB) so the up-to-8 GiB DRAM
 * window reported by boot0 (monitor header +0x2C) can be mapped; the
 * same context uses 6 xlat tables and an 8-entry mmap array.
 */
#define PLAT_VIRT_ADDR_SPACE_SIZE	(1ULL << 35)
#define PLAT_PHY_ADDR_SPACE_SIZE	(1ULL << 35)
#define MAX_XLAT_TABLES			6

#define CACHE_WRITEBACK_SHIFT		6
#define CACHE_WRITEBACK_GRANULE		(1 << CACHE_WRITEBACK_SHIFT)

#define MAX_STATIC_MMAP_REGIONS		6
#define MAX_MMAP_REGIONS		(8 + MAX_STATIC_MMAP_REGIONS)

/* optional BL32 (OP-TEE): optee_sun60iw2p1.bin load address */
#ifndef BL32_BASE
#endif

/* Non-secure entry */
#define PLAT_SUNXI_NS_IMAGE_OFFSET	(0x200000)
#ifndef PRELOADED_BL33_BASE
#define PRELOADED_BL33_BASE		SUNXI_BL33_BASE
#endif

/* Console: UART0 @ 0x02500000, 24MHz, 115200 (matches bootloader) */
#define SUNXI_UART0_BASE		0x02500000UL
#define SUNXI_UART0_CLK_IN_HZ		24000000UL
#define SUNXI_UART0_BAUDRATE		115200UL
#define SUNXI_CONSOLE_BASE		SUNXI_UART0_BASE

#define SUNXI_OSC24M_CLK_IN_HZ		24000000

#endif /* PLATFORM_DEF_H */
