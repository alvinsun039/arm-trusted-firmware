/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Vendor DRM / secure-DRAM support:
 *
 *   - the DRM window is read from the display registers
 *   - the region is validated and cached, and the suspend path uses it
 *   - FF60 ARM_SVC_SUNXI_DRM_OPS command 1 is
 *     "sunxi_drm_copy_from_unsafe"
 *   0x0DA24 - boot-time secure DRAM / DRM region table collector,
 *             called at 0x4434 right after sunxi_system_restore();
 *             consumed by the FES fes_up handler (0x1384) which
 *             answers zeroes for ranges overlapping the table.
 *
 * Register facts:
 *   SID/DRM control word   0x030060A0  (bit0 = DRM window active)
 *   DRM window start/end   0x0A000060 / 0x0A000064
 *     (field = addr[35:15] << 12 -> recovered as (reg >> 12) << 15)
 *   secure DRAM region n   0x0A000110 + n * 0x10, n = 0..13
 *     +0x00 ctrl (address bits [35:15], masked 0xFFFF8000)
 *     +0x04 high (address bits above 2^32)
 *     +0x08 size class: (reg >> 1) & 0x3F; class <= 13 or bit29 set
 *           -> entry unused; size = 0x8000 << (class - 14)
 *     address = (ctrl & 0xFFFF8000) | (high << 32) + 0x40000000
 */

#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

#include <sunxi_def.h>
#include <sunxi_mmap.h>
#include <sunxi_private.h>

#define SUNXI_DRM_SID_REG		0x030060A0U
#define SUNXI_DRM_START_REG		0x0A000060U
#define SUNXI_DRM_END_REG		0x0A000064U
#define SUNXI_DRAM_REGION_BASE		0x0A000110U
#define SUNXI_DRAM_REGION_COUNT		14U
#define SUNXI_DRAM_REGION_STRIDE	0x10U
#define SUNXI_DRAM_BASE			0x40000000ULL
#define SUNXI_DRM_MAX_REGIONS		16U

/* Both raw register values are cached in write-only
 * globals (0x48040CF4/0x48040CF8); kept here for parity. */
static uint32_t sunxi_drm_start_cache;
static uint32_t sunxi_drm_end_cache;

static uint64_t sunxi_sec_regions[SUNXI_DRM_MAX_REGIONS][2];
static unsigned int sunxi_sec_region_count;

/* DRAM end used by the bounds guards below (same source as the MMU
 * window: monitor header word at +0x2C in megabytes). */
static uint64_t sunxi_dram_end(void)
{
	uint32_t mb = mmio_read_32(SUNXI_BL31_BASE + 0x2CU);

	if ((mb >= 16U) && (mb <= 8192U))
		return SUNXI_DRAM_BASE + ((uint64_t)mb << 20);

	return SUNXI_DRAM_BASE + 0x80000000ULL;	/* 2 GiB fallback */
}

/* Registered safety deviation: every buffer the DRM services touch must
 * lie inside DRAM.  Legitimate callers (display buffers) always do. */
int sunxi_drm_ptr_ok(uintptr_t p, uint64_t len)
{
	uint64_t dram_end = sunxi_dram_end();

	if (p < SUNXI_DRAM_BASE)
		return 0;
	if (p > dram_end)
		return 0;
	if (len > (dram_end - p))
		return 0;

	return 1;
}

/* : start = (reg >> 12) << 15, size = end - start */
static void sunxi_drm_read_window(uint64_t *start, uint64_t *size)
{
	uint32_t s = mmio_read_32(SUNXI_DRM_START_REG);
	uint32_t e = mmio_read_32(SUNXI_DRM_END_REG);
	uint64_t base = ((uint64_t)(s >> 12)) << 15;

	if (start != NULL)
		*start = base;
	if (size != NULL)
		*size = (((uint64_t)(e >> 12)) << 15) - base;
}

/* called from the suspend path */
void sunxi_drm_check_region(void)
{
	if ((mmio_read_32(SUNXI_DRM_SID_REG) & 1U) == 0U)
		return;

	sunxi_drm_start_cache = mmio_read_32(SUNXI_DRM_START_REG);
	sunxi_drm_end_cache = mmio_read_32(SUNXI_DRM_END_REG);

	if ((sunxi_drm_start_cache == 0U) || (sunxi_drm_end_cache == 0U))
		NOTICE("Invalid DRM region config: DRM_START_ADDR_REG = 0x%x, DRM_END_ADDR_REG = 0x%x\n",
		       sunxi_drm_start_cache, sunxi_drm_end_cache);
}

/* : build the secure-DRAM + DRM region table */
void sunxi_secure_mem_init(void)
{
	unsigned int i;
	uint64_t drm_start = 0U;
	uint64_t drm_size = 0U;

	memset(sunxi_sec_regions, 0, sizeof(sunxi_sec_regions));
	sunxi_sec_region_count = 0U;

	for (i = 0U; i < SUNXI_DRAM_REGION_COUNT; i++) {
		uint32_t ctrl = mmio_read_32(SUNXI_DRAM_REGION_BASE +
					     i * SUNXI_DRAM_REGION_STRIDE);
		uint32_t high = mmio_read_32(SUNXI_DRAM_REGION_BASE +
					     i * SUNXI_DRAM_REGION_STRIDE + 4U);
		uint32_t cls_reg = mmio_read_32(SUNXI_DRAM_REGION_BASE +
					     i * SUNXI_DRAM_REGION_STRIDE + 8U);
		unsigned int cls = (cls_reg >> 1) & 0x3FU;
		uint64_t size;
		uint64_t addr;

		if (cls <= 0xDU)
			continue;
		if ((cls_reg & BIT(29)) != 0U)
			continue;

		size = 0x8000ULL << (cls - 0xEU);
		addr = ((uint64_t)(ctrl & 0xFFFF8000U)) |
		       ((uint64_t)high << 32);
		addr += SUNXI_DRAM_BASE;

		sunxi_sec_regions[sunxi_sec_region_count][0] = addr;
		sunxi_sec_regions[sunxi_sec_region_count][1] = size;
		NOTICE("SEC mem [%d]: addr = 0x%llx, size = 0x%llx\n",
		       (int)sunxi_sec_region_count,
		       (unsigned long long)addr, (unsigned long long)size);
		sunxi_sec_region_count++;
	}

	if ((mmio_read_32(SUNXI_DRM_SID_REG) & 1U) != 0U)
		sunxi_drm_read_window(&drm_start, &drm_size);

	if (drm_size != 0U) {
		sunxi_sec_regions[sunxi_sec_region_count][0] = drm_start;
		sunxi_sec_regions[sunxi_sec_region_count][1] = drm_size;
		NOTICE("DRM mem [%d]: addr = 0x%llx, size = 0x%llx\n",
		       (int)sunxi_sec_region_count,
		       (unsigned long long)drm_start,
		       (unsigned long long)drm_size);
		sunxi_sec_region_count++;
	}
}

/* -0x13C0: FES fes_up overlap test against the table */
int sunxi_drm_range_is_secure(uint64_t addr, uint64_t size)
{
	uint64_t end = addr + size - 1U;
	unsigned int i;

	for (i = 0U; i < sunxi_sec_region_count; i++) {
		uint64_t region = sunxi_sec_regions[i][0];
		uint64_t region_size = sunxi_sec_regions[i][1];

		if (region_size == 0U)
			continue;
		if (addr < region)
			continue;
		if (addr < region + region_size)
			return 1;
		if (end < region)
			continue;
		if (end < region + region_size)
			return 1;
	}

	return 0;
}

/* : FF60 command 1 */
int sunxi_drm_copy_from_unsafe(uint64_t dst, uint64_t src, uint64_t size)
{
	uint64_t start = 0U;
	uint64_t wsize = 0U;

	if ((mmio_read_32(SUNXI_DRM_SID_REG) & 1U) != 0U)
		sunxi_drm_read_window(&start, &wsize);

	/*
	 * Registered safety deviation: the vendor BSP skips every
	 * check when the DRM window is inactive (SID bit0=0 degenerates
	 * the window to {0,0}, whose test accepts almost any destination)
	 * and it never validates the source, which makes FF60 a
	 * non-secure -> EL3 read/write primitive.  Keep both buffers
	 * inside DRAM; display buffers always are.
	 */
	if (!sunxi_drm_ptr_ok((uintptr_t)dst, size)) {
		goto out;
	}
	if (!sunxi_drm_ptr_ok((uintptr_t)src, size)) {
		NOTICE("DRM: copy source 0x%llx size 0x%llx out of DRAM\n",
		       (unsigned long long)src, (unsigned long long)size);
		return -1;
	}

	/* -0x0B4D8 window enforcement */
	if ((dst < start) || ((start + wsize - 1U) < (dst + size - 1U)))
		goto out;

	memcpy((void *)(uintptr_t)dst, (const void *)(uintptr_t)src,
	       (size_t)size);
	return 0;

out:
	NOTICE("%s(): dst 0x%llx is out of boundary\n",
	       "sunxi_drm_copy_from_unsafe", (unsigned long long)dst);
	return -1;
}