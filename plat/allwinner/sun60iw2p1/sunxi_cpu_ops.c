/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CPU power control for the A733: per-core PPU state, the AArch64/entry
 * control bits and RVBAR, driven from the PSCI CPU ops.
 *
 * Registers from the A733 User Manual:
 *   CPU_SUBSYS_CTRL @0x08000000  (5.1.5): CLU0_CPUn_CTRL  @+0x1000+n*0x1000
 *     [0] AA64NAA32 (sampled at reset), RVBARADDRn_L @+4, H @+8  (5.1.6)
 *   CPUIDLE @0x07050000: per-core PPU PDSTATE @0x07052000+n*0x1000+8
 *     (0 = OFF, 8 = ON), per-core ctrl word @0x07050220+n*4.
 */

#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <lib/utils_def.h>
#include <plat/common/platform.h>

#include <sunxi_mmap.h>
#include <sunxi_private.h>

/* per-core register blocks */
#define SUNXI_CPUN_CTRL_REG(n)		(SUNXI_CPU_SUBSYS_BASE + 0x1000 + (n) * 0x1000)
#define SUNXI_CPUN_CTRL_AA64NAA32	BIT(0)
#define SUNXI_RVBAR_LO_REG(n)		(SUNXI_CPU_SUBSYS_BASE + 0x1004 + (n) * 0x1000)
#define SUNXI_RVBAR_HI_REG(n)		(SUNXI_CPU_SUBSYS_BASE + 0x1008 + (n) * 0x1000)

#define SUNXI_PPU_PDSTATE(n)		(SUNXI_CPUIDLE_BASE + 0x2000 + (n) * 0x1000 + 0x8)
#define SUNXI_PPU_PDSTATE_ON		0x8U
#define SUNXI_PPU_PDSTATE_MASK		0xfU

/* per-core CPUIDLE control words :
 *   0x200 bit0 = power-on hold, bit1 = cluster/suspend assist
 *   0x220 bit0 = power-on hold                                     */
#define SUNXI_CPUIDLE_CONTROL(n)	(SUNXI_CPUIDLE_BASE + 0x200 + (n) * 4)
#define SUNXI_CPUIDLE_ASSIST_BIT	BIT(1)
#define SUNXI_CPUIDLE_HOLD_BIT		BIT(0)
#define SUNXI_CPUIDLE_CTRL(n)		(SUNXI_CPUIDLE_BASE + 0x220 + (n) * 4)

/* bound (ms) for the PPU power-on handshake waits */
#define SUNXI_PPU_WAIT_MS		100U
/* iteration-count bound for the same waits - the system counter
 * can stall with the cluster power transient, which makes the ms-bound
 * unreachable. */
#define SUNXI_PPU_WAIT_ITERS		4000000U

static unsigned int sunxi_core_index(u_register_t mpidr)
{
	return (unsigned int)MPIDR_AFFLVL1_VAL(mpidr);
}

bool sunxi_cpu_is_powered(unsigned int core)
{
	return (mmio_read_32(SUNXI_PPU_PDSTATE(core)) & SUNXI_PPU_PDSTATE_MASK)
	       == SUNXI_PPU_PDSTATE_ON;
}

/* FF42 helper (raw PPU byte) was retired: the firmware's FF42 returns a
 * boolean via 0xC650, handled in sunxi_sip_svc.c. */

/*
 * Per-core CPU errata, mirroring the vendor monitor.
 *
 * the vendor BSP's errata name table (0x48012600) lists exactly the set enabled
 * in platform.mk (dsu_798953, dsu_936184, the A55 set and the A76 set).
 * This boot chain has no BL1 and TF-A's generic code never invokes the
 * cpu_ops reset functions in a BL31-only build, so platform code calls
 * them on every core: cold boot (sunxi_bl31_setup) and every warm
 * power-up (sunxi_pwr_domain_on_finish).
 *
 * The reset functions are stack-free: they keep the link register in x15
 * and only clobber x8-x15 (see cpu_reset_prologue/cpu_reset_func_end).
 * Without the DSU workarounds (disabling DSU clock gating) a freshly
 * powered-on core wedges the interconnect on its first shareable access
 * and the whole system dies silently.
 */
extern void cortex_a55_reset_func(void);
extern void cortex_a76_reset_func(void);

void sunxi_apply_cpu_errata(void)
{
	u_register_t midr = read_midr();

	if ((midr & 0xFF0FFFF0U) == 0x410FD050U) {	/* Cortex-A55 */
		cortex_a55_reset_func();
	} else if ((midr & 0xFF0FFFF0U) == 0x410FD0B0U) {	/* A76 */
		cortex_a76_reset_func();
	}
}

void sunxi_cpu_on(u_register_t mpidr)
{
	unsigned int core = sunxi_core_index(mpidr);
	uintptr_t entry = sec_entrypoint_get();

	VERBOSE("PSCI: power on core %d to 0x%lx\n", core, entry);

	/* target AArch64 reset state: AA64NAA32 = 1 (manual 5.1.6.6,
	 * sampled only during reset) */
	mmio_setbits_32(SUNXI_CPUN_CTRL_REG(core), SUNXI_CPUN_CTRL_AA64NAA32);

	/* set the reset vector (RVBARADDR[39:0], manual 5.1.6.7/.8);
	 *  writes the high word as 0 unconditionally */
	mmio_write_32(SUNXI_RVBAR_LO_REG(core), (uint32_t)(entry & 0xffffffffU));
	mmio_write_32(SUNXI_RVBAR_HI_REG(core), 0U);

	unsigned int iters = 0U;
	uint32_t pdstate;
	uint64_t wdead;

	wdead = read_cntpct_el0() + (SUNXI_PPU_WAIT_MS * 24000ULL);
	pdstate = mmio_read_32(SUNXI_PPU_PDSTATE(core));
	while ((pdstate & SUNXI_PPU_PDSTATE_MASK) != 0U) {
		if ((++iters >= SUNXI_PPU_WAIT_ITERS) ||
		    (read_cntpct_el0() > wdead)) {
			break;
		}
		pdstate = mmio_read_32(SUNXI_PPU_PDSTATE(core));
	}

	iters = 0U;
	wdead = read_cntpct_el0() + (SUNXI_PPU_WAIT_MS * 24000ULL);
	mmio_setbits_32(SUNXI_CPUIDLE_CTRL(core), SUNXI_CPUIDLE_HOLD_BIT);
	mmio_setbits_32(SUNXI_CPUIDLE_CONTROL(core), SUNXI_CPUIDLE_HOLD_BIT);

	pdstate = mmio_read_32(SUNXI_PPU_PDSTATE(core));
	while ((pdstate & SUNXI_PPU_PDSTATE_MASK) != SUNXI_PPU_PDSTATE_ON) {
		if ((++iters >= SUNXI_PPU_WAIT_ITERS) ||
		    (read_cntpct_el0() > wdead)) {
			break;
		}
		pdstate = mmio_read_32(SUNXI_PPU_PDSTATE(core));
	}

	mmio_clrbits_32(SUNXI_CPUIDLE_CONTROL(core), SUNXI_CPUIDLE_HOLD_BIT);
	mmio_clrbits_32(SUNXI_CPUIDLE_CTRL(core), SUNXI_CPUIDLE_HOLD_BIT);

	dsb();
	isb();

}

void sunxi_cpu_off(void)
{
	unsigned int core = plat_my_core_pos();

	/*
	 * Vendor parity (0x8624): clear the CPUIDLE CTRL hold (0x220 bit0)
	 * and raise the cluster assist (0x200 bit1).  The vendor does NOT
	 * clear 0x200 bit0 here; that write makes the PPU remove the core
	 * immediately, before the generic PSCI tail has committed
	 * AFF_STATE_OFF and released the power-domain locks.
	 */
	mmio_clrbits_32(SUNXI_CPUIDLE_CTRL(core), SUNXI_CPUIDLE_HOLD_BIT);
	mmio_setbits_32(SUNXI_CPUIDLE_CONTROL(core), SUNXI_CPUIDLE_ASSIST_BIT);
	dsb();
	isb();
	/*
	 * No console output here: during system suspend the UART clock is
	 * already gated and a NOTICE would block in the 16550 THRE poll
	 * long enough for the PPU to remove this core's power before the
	 * PSCI bookkeeping (AFF_STATE_OFF, lock release) has completed.
	 */
}

/* No vendor counterpart: the vendor BSP has no per-core "power off others"
 * helper; removed to keep the port free of unknown-behaviour code. */

/*
 * Cluster-level power assist around system suspend :
 * asserts/releses the cluster PPU dependency so the boot core can be
 * re-absorbed while the arisc drains the rest of the cluster.
 */
void sunxi_cluster_power_assist(unsigned int cluster, unsigned int core)
{
	/* : set 0x200 bit1 for the core */
	(void)cluster;
	mmio_setbits_32(SUNXI_CPUIDLE_CONTROL(core), SUNXI_CPUIDLE_ASSIST_BIT);
}

/*
 * DMA channel state save/restore around suspend, mirroring the vendor BSP
 * 0x5C0C (save 8 x channel+0x200 status into a BSS table) and 0x5BBC
 * (restore: write 0xFFFF to +0x280 then the saved status to +0x200).
 * The channel pointer table lives in BSS; it is empty in this build
 * because the BL31 has no active DMA users, so both routines are
 * no-ops until a DMA driver registers its channels - structurally
 * identical to the vendor BSP.
 */
#define SUNXI_DMA_CHANNELS_MAX	8

static uintptr_t dma_channels[SUNXI_DMA_CHANNELS_MAX];
static uint16_t dma_ch_status[SUNXI_DMA_CHANNELS_MAX];

void sunxi_dma_save(void)
{
	unsigned int i;

	for (i = 0U; i < SUNXI_DMA_CHANNELS_MAX; i++) {
		uintptr_t ch = dma_channels[i];

		if (ch == 0U)
			continue;
		dma_ch_status[i] = (uint16_t)(mmio_read_32(ch + 0x200) & 0xffffU);
	}
}

void sunxi_dma_restore(void)
{
	unsigned int i;

	for (i = 0U; i < SUNXI_DMA_CHANNELS_MAX; i++) {
		uintptr_t ch = dma_channels[i];

		if (ch == 0U)
			continue;
		mmio_write_32(ch + 0x280, 0xffffU);
		mmio_write_32(ch + 0x200, dma_ch_status[i]);
	}
}
