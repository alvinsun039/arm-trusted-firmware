/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <sunxi_def.h>
#include <sunxi_mmap.h>
#include <sunxi_private.h>

/*
 * Platform static map, mirroring the table the vendor BSP keeps at
 * 0x4800F158 (mmap_add walks it until the granularity terminator):
 *   0x00000000-0x40000000  device RW secure XN  (peripherals + SRAM)
 *   0x48000000 + 2MiB      normal RW secure XN  (BL31 image + E902 window)
 *   0x48200000 + 16KiB     device RW secure XN
 * The DRAM and conditional OP-TEE regions are appended dynamically below,
 * exactly like  does.
 */
static const mmap_region_t sunxi_mmap[MAX_STATIC_MMAP_REGIONS + 1] = {
	MAP_REGION_FLAT(0x00000000UL, 0x40000000UL,
			MT_DEVICE | MT_RW | MT_SECURE | MT_EXECUTE_NEVER),
	MAP_REGION_FLAT(SUNXI_BL31_BASE, 0x00200000UL,
			MT_RW_DATA | MT_SECURE | MT_EXECUTE_NEVER),
	MAP_REGION_FLAT(0x48200000UL, 0x00004000UL,
			MT_DEVICE | MT_RW | MT_SECURE | MT_EXECUTE_NEVER),
	{},
};

unsigned int plat_get_syscnt_freq2(void)
{
	return SUNXI_OSC24M_CLK_IN_HZ;
}

void sunxi_configure_mmu_el3(int flags)
{
	mmap_add_region(BL_CODE_BASE, BL_CODE_BASE,
			BL_CODE_END - BL_CODE_BASE,
			MT_CODE | MT_SECURE);
	mmap_add_region(BL_CODE_END, BL_CODE_END,
			BL_END - BL_CODE_END,
			MT_RW_DATA | MT_SECURE);
#if SEPARATE_CODE_AND_RODATA
	mmap_add_region(BL_RO_DATA_BASE, BL_RO_DATA_BASE,
			BL_RO_DATA_END - BL_RO_DATA_BASE,
			MT_RO_DATA | MT_SECURE);
#endif
#if SEPARATE_NOBITS_REGION
	mmap_add_region(BL_NOBITS_BASE, BL_NOBITS_BASE,
			BL_NOBITS_END - BL_NOBITS_BASE,
			MT_RW_DATA | MT_SECURE);
#endif

	/*
	 * DRAM window:  takes the size from the monitor
	 * header word at +0x2C (megabytes, filled in by boot0).  Keep a
	 * sanity range so a stale/absent field cannot poison the
	 * translation tables.
	 */
	{
		uint32_t dram_mb = mmio_read_32(SUNXI_BL31_BASE + 0x2CU);
		size_t dram_size = 0x80000000ULL;

		/*  prints the raw header value first */
		NOTICE("BL31: dram size is %llu bytes\n",
		       (unsigned long long)((uint64_t)dram_mb << 20));

		if (dram_mb >= 16U && dram_mb <= 8192U)
			dram_size = (size_t)dram_mb << 20;

		mmap_add_region(SUNXI_DRAM_BASE, SUNXI_DRAM_BASE, dram_size,
				MT_RW_DATA | MT_NS | MT_EXECUTE_NEVER);
	}

	/* OP-TEE windows when boot0 handed over a secure-os base (+0x18) */
	if (mmio_read_32(SUNXI_BL31_BASE + 0x18U) != 0U) {
		mmap_add_region(0x48200000UL, 0x48200000UL, 0x00E00000UL,
				MT_MEMORY | MT_RW);
		mmap_add_region(0x48200000UL, 0x48200000UL, 0x00400000UL,
				MT_MEMORY | MT_RW | MT_NS);
	}

	mmap_add(sunxi_mmap);
	init_xlat_tables();

	enable_mmu_el3(0);
}

/*
 * SoC revision:  reads the SYSCTRL VER register at
 * 0x03000024, toggling bit15 around the sample and returning the high
 * 16-bit chip revision (the errata switch @0x4438 compares this value;
 * 0x1689/0x1718 select the CCU workarounds).  SID+0x200 (0x03006200,
 * read by the secure-boot path @0x5A80) is a different word.
 */
uint16_t sunxi_read_soc_id(void)
{
	uint32_t v;

	v = mmio_read_32(SUNXI_SYSCTRL_BASE + 0x24U);
	mmio_write_32(SUNXI_SYSCTRL_BASE + 0x24U, v | 0x8000U);
	v = mmio_read_32(SUNXI_SYSCTRL_BASE + 0x24U);
	mmio_write_32(SUNXI_SYSCTRL_BASE + 0x24U, v & ~0x8000U);

	return (uint16_t)(v >> 16);
}

static int32_t ccu_pll_idx = -1;
static int32_t ccu_pll_rate = -1;
static int32_t ccu_pll_base = -1;

void sunxi_ccu_restore(void)
{
	uint32_t idx, base, reg, p, n, f, t;
	unsigned int i;

	if (ccu_pll_idx == -1) {
		ccu_pll_idx = 6;
		for (i = 0U; i < 7U; i++) {
			if ((mmio_read_32(0x02002E00U + 4U * i) & 1U) != 0U) {
				ccu_pll_idx = (int32_t)i;
				break;
			}
		}
	}

	idx = (uint32_t)ccu_pll_idx;
	base = (0x9400U + idx) << 10;

	if (ccu_pll_rate == -1) {
		/* enable, sample P/N, toggle update, disable (0xC904-0xC964) */
		mmio_setbits_32(base + 0xA4U, BIT(1));
		reg = mmio_read_32(0x02002538U);
		p = (reg & 0x1FU) + 1U;
		t = mmio_read_32(base + 0x0CU);
		mmio_write_32(base + 0x0CU, t | 0x80U);
		n = ((mmio_read_32(base + 4U) << 8) |
		     (mmio_read_32(base) & 0xFFU)) << 4;
		mmio_write_32(base + 0x0CU, t & ~0x80U);
		mmio_clrbits_32(base + 0xA4U, BIT(1));

		switch ((reg >> 24) & 0x7U) {
		case 3U:
			f = 0x23C34600U / p;
			ccu_pll_base = (int32_t)f;
			ccu_pll_rate = (int32_t)(f / n);
			break;
		case 4U:
			f = 0x1C93800U / p;
			ccu_pll_base = (int32_t)f;
			ccu_pll_rate = (int32_t)(f / n);
			break;
		case 0U:
			f = 0x16E3600U / p;
			ccu_pll_base = (int32_t)f;
			ccu_pll_rate = (int32_t)(f / n);
			if (ccu_pll_rate == (int32_t)0x1C2B8)
				ccu_pll_rate = (int32_t)0x1C200;
			break;
		default:
			ccu_pll_base = (int32_t)0x16E3600U;
			ccu_pll_rate = (int32_t)0x1C200;
			break;
		}
	}

	/* re-arm the PLL state word (0xC99C-0xC9E4 tail) */
	t = 0x02002E00U + 4U * idx;
	reg = mmio_read_32(t);
	mmio_write_32(t, reg & ~0x1U);
	reg = mmio_read_32(t);
	mmio_write_32(t, reg & ~0x10000U);
	mmio_write_32(t, (reg & ~0x10000U) | 0x10000U);
	reg = mmio_read_32(t);
	mmio_write_32(t, reg | 0x1U);
}
