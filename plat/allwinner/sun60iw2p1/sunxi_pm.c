/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * PSCI operations for the A733, modeled after the ops table recovered
 * from the vendor BSP (plat_pm.c, addresses 0x08224-0x088B4).
 *
 * Alignment notes:
 *  - cpu_on uses the CPUIDLE/PPU + RVBAR handshake exactly like the
 *    firmware (0xC674/0xC87C/0xC6F0).
 *  - system suspend hands the CPU_OP over to the arisc (0x19F0) and the
 *    resume path runs the full GIC/DMA/NSI/CCU/SMC/0xC794 restore chain.
 */

#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <drivers/arm/gicv3.h>
#include <lib/el3_runtime/aarch64/context.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/el3_runtime/cpu_data.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <lib/utils_def.h>
#include <drivers/delay_timer.h>
#include <plat/common/platform.h>

#include <sunxi_private.h>

/* sec_entrypoint handed over by the PSCI core in plat_setup_psci_ops() */
static uintptr_t sunxi_sec_entrypoint;

/* GIC distributor context saved across system suspend  */
static gicv3_dist_ctx_t sunxi_dist_ctx;
/* Redistributor (SGI/PPI) context.  The resume path used to call
 * gicv3_rdistif_init(), which runs gicv3_ppi_sgi_config_defaults() -
 * that DISABLES every SGI and PPI of the resuming core (GICR_ICENABLER0
 * = ~0) and nothing ever re-enables them: the arch-timer tick PPI 27
 * dies (jiffies frozen) and all IPIs die, so the first timeout-based
 * sleep of the resume never wakes.  TF-A's canonical suspend/resume
 * pair (gicv3_rdistif_save / gicv3_rdistif_init_restore) is used
 * instead. */
static gicv3_redist_ctx_t sunxi_rdist_ctx;

/* IMPDEF EL3 register  */
static inline void sunxi_imd_el3_write(uint64_t v)
{
	__asm__ volatile("msr s3_0_c15_c3_6, %0" : : "r"(v));
}

static inline void sunxi_cntfrq_arm(void)
{
	__asm__ volatile("msr cntfrq_el0, %0" : : "r"(0x16E3600UL));
}

/*
 * Always-on R-domain watchdog (base 0x07021000).  The ARISC's
 * super-standby powers the SYS domain (and the normal WDT) down, so this
 * is the only timer that survives the sleep: hardening net for the
 * system-suspend path.  If the ARISC never resumes the boot core the
 * watchdog resets the whole SoC; boot0 then re-runs while the RTC record
 * register (RTC+0x10C) still holds the ARISC's last standby stage, which
 * the next BL31 cold boot prints as "arisc last state flag".
 *
 * Register map (sun60iw2p1, key 0x16AA in bits 31:16):
 *   +0x00: IRQ control
 *   +0x14: interval select [7:4], clock source [8] (1 = LOSC, always-on),
 *          reset target [1:0] (1 = reset SYS)
 *   +0x18: enable [0]
 * Interval table (vendor ARISC watchdog driver, watchdog_i.h):
 *   0=.5s 1=1s 2=2s 3=3s 4=4s 5=5s 6=6s 7=8s 8=10s 9=12s 10=14s
 *   11=16s; 12..15 are invalid and leave the watchdog stopped.
 */
#define SUNXI_R_WDOG_BASE	0x07021000U
#define SUNXI_R_WDOG_KEY	0x16AA0000U
#define SUNXI_R_WDOG_SEL_12S	9U
#define SUNXI_R_WDOG_SEL_16S	11U


#define SUNXI_MRS(name, reg) \
	static inline uint64_t sunxi_read_##name(void) \
	{ uint64_t v; __asm__ volatile("mrs %0, " #reg : "=r"(v)); return v; }
#define SUNXI_MSR(name, reg) \
	static inline void sunxi_write_##name(uint64_t v) \
	{ __asm__ volatile("msr " #reg ", %0" :: "r"(v)); }

struct sunxi_el2_state {
	uint64_t hcr;
	uint64_t cptr;
	uint64_t cnthctl;
	uint64_t cntvoff;
	uint64_t vpidr;
	uint64_t vmpidr;
	uint64_t vttbr;
	uint64_t mdcr;
	uint64_t hstr;
	uint64_t cnthp_ctl;
};
static struct sunxi_el2_state __unused sunxi_el2_backup;
static bool sunxi_el2_valid;

SUNXI_MRS(hcr, hcr_el2)
SUNXI_MRS(cptr, cptr_el2)
SUNXI_MRS(cnthctl, cnthctl_el2)
SUNXI_MRS(cntvoff, cntvoff_el2)
SUNXI_MRS(vpidr, vpidr_el2)
SUNXI_MRS(vmpidr, vmpidr_el2)
SUNXI_MRS(vttbr, vttbr_el2)
SUNXI_MRS(mdcr, mdcr_el2)
SUNXI_MRS(hstr, hstr_el2)
SUNXI_MRS(cnthp_ctl, cnthp_ctl_el2)
SUNXI_MSR(hcr, hcr_el2)
SUNXI_MSR(cptr, cptr_el2)
SUNXI_MSR(cnthctl, cnthctl_el2)
SUNXI_MSR(cntvoff, cntvoff_el2)
SUNXI_MSR(vpidr, vpidr_el2)
SUNXI_MSR(vmpidr, vmpidr_el2)
SUNXI_MSR(vttbr, vttbr_el2)
SUNXI_MSR(mdcr, mdcr_el2)
SUNXI_MSR(hstr, hstr_el2)
SUNXI_MSR(cnthp_ctl, cnthp_ctl_el2)

/*
 * vendor-parity per-CPU wake fixups.  Replay the EL2 state
 * captured at suspend (the vendor monitor does the same from its per-CPU
 * wake init).  Called from bl31_warm_entrypoint for every warm boot.
 */
void sunxi_warmboot_fixups(void)
{
	/* vendor-parity per-CPU EL2 wake init .  A pKVM-configured kernel
	 * (kvm-arm.mode=protected) resumes secondaries at EL1 whose hvc
	 * calls enter the EL2 hypervisor image; the power cycle resets
	 * every EL2 sysreg, so the BL31 must put the baseline back or the
	 * secondary wedges right after the warm-path completes. */
	u_register_t midr = read_midr();
	u_register_t mpidr = read_mpidr_el1();
	u_register_t mdcr = (read_pmcr_el0() >> 11) & 0x1FU;

	__asm__ volatile(
		"msr	sctlr_el2, %0\n"
		/* vendor-parity - the reference 0x48D0 writes hcr_el2
		 * only on its SCR.HVC==0 branch; on the HVC==1 branch (the
		 * Linux/SMC case) it restores sctlr_el2 alone.  Our
		 * unconditional 0x30080000000 (TGE|E2H|RW) is the last
		 * structural delta on this path - TGE diverts IRQ delivery
		 * away from the plain EL1/EL2-unused setup. */
		"msr	cptr_el2, %1\n"
		"msr	cnthctl_el2, %2\n"
		"msr	cntvoff_el2, xzr\n"
		"msr	vpidr_el2, %3\n"
		"msr	vmpidr_el2, %4\n"
		"msr	vttbr_el2, xzr\n"
		"msr	mdcr_el2, %5\n"
		"msr	hstr_el2, xzr\n"
		"msr	cnthp_ctl_el2, xzr\n"
		"isb\n"
		:
		: "r"(0x30C50830UL), "r"(0x33FFUL),
		  "r"(3UL), "r"(midr), "r"(mpidr),
		  "r"(mdcr | 0x4020000UL)
		: "memory");
	sunxi_el2_valid = false;
}

/*
 * Bring-up tracing for the system-suspend resume window.  No console print
 * is possible between the ARISC hand-off and the kernel's console resume
 * (the 16550 putc spins on a gated UART and can wedge the resume), so the
 * only way to tell how far BL31 got is to leave stage markers in the RTC's
 * general-purpose registers - the ARISC's periodic register dump reads
 * them out over its own console.  Off by default: release builds carry no
 * debug stores.
 */
#define SUNXI_RTC_GPR(n)		(0x07090000U + 0x100U + 4U * (n))
#define SUNXI_RTC_S3_STAGE_REG		SUNXI_RTC_GPR(5)	/* 0x07090114 */
#define SUNXI_RTC_S3_TRACE_REG		SUNXI_RTC_GPR(6)	/* 0x07090118 */

#ifndef SUNXI_S3_TRACE
#define SUNXI_S3_TRACE			0
#endif

#if SUNXI_S3_TRACE
#define sunxi_s3_trace(reg, stage)	mmio_write_32((reg), (stage))
#else
#define sunxi_s3_trace(reg, stage)	do { } while (0)
#endif

#define SUNXI_NOTICE(...)					\
	do {							\
		NOTICE(__VA_ARGS__);				\
	} while (0)

/* clear the cluster assist bit  */
static void sunxi_cpu_assist_clear(unsigned int core)
{
	mmio_clrbits_32(SUNXI_CPUIDLE_BASE + 0x200U + core * 4U, BIT(1));
}

/*
 * DRAM-controller capacity registers (SMC0 +0x60/+0x64, mirrored into
 * SMC1 by the vendor BSP).  The arisc resume flow reprograms the
 * controller, so BL31 snapshots them on suspend and restores on finish.
 * Gated by bit0 of the SID control word .
 */
#define SUNXI_SMC0_MCAP_A		(SUNXI_SMC0_BASE + 0x60)
#define SUNXI_SMC0_MCAP_B		(SUNXI_SMC0_BASE + 0x64)
#define SUNXI_SMC1_MCAP_A		(SUNXI_SMC0_BASE + 0x10060)
#define SUNXI_SMC1_MCAP_B		(SUNXI_SMC0_BASE + 0x10064)
#define SUNXI_SID_CTRL_FLAG		(SUNXI_SID_BASE + 0xA0)
#define SUNXI_SID_FLAG_DRAM_READY	BIT(0)

uint32_t sunxi_smc_mcap_a;
uint32_t sunxi_smc_mcap_b;
uint32_t sunxi_nsi_word_0;
uint32_t sunxi_nsi_word_1;
uint32_t sunxi_standby_result;

/*
 * Restore the DRAM controller capacity words after the arisc standby.
 * The standby reprograms the controller, so the values snapshotted in
 * sunxi_pwr_domain_suspend() are written back to both channels.  The
 * SID flag gates this exactly like the snapshot.
 */
static void sunxi_smc_cap_restore(void)
{
	if ((mmio_read_32(SUNXI_SID_CTRL_FLAG) &
	     SUNXI_SID_FLAG_DRAM_READY) == 0U) {
		return;
	}

	mmio_write_32(SUNXI_SMC0_MCAP_A, sunxi_smc_mcap_a);
	mmio_write_32(SUNXI_SMC0_MCAP_B, sunxi_smc_mcap_b);
	mmio_write_32(SUNXI_SMC1_MCAP_A, sunxi_smc_mcap_a);
	mmio_write_32(SUNXI_SMC1_MCAP_B, sunxi_smc_mcap_b);
}

uintptr_t sec_entrypoint_get(void)
{
	return sunxi_sec_entrypoint;
}

static int sunxi_pwr_domain_on(u_register_t mpidr)
{
	sunxi_cpu_on(mpidr);
	return PSCI_E_SUCCESS;
}

/*
 * System restore, mirroring  byte for byte: SPC master
 * enables, TZMA/S_SPC defaults, PRCM security switch, the two SEC_REG
 * whitelist words (CE 0x04601020 / DMAC1 0x04024020), CCU window
 * registers and the TIMER0 control word.  Called at boot (0x4430) and
 * after the arisc resume flow reprograms the power/clock trees (0x895C).
 */
void sunxi_system_restore(void)
{
	uintptr_t a;

	for (a = 0x02054004UL; a != 0x020540E4UL; a += 16UL)
		mmio_write_32(a, 0xFFFFFFFFUL);
	for (a = 0x02054104UL; a <= 0x02054174UL; a += 16UL)
		mmio_write_32(a, 0xFFFFFFFFUL);
	mmio_write_32(0x07003000UL, 0);	/* S_TZMA */
	mmio_write_32(0x07003004UL, 0);
	mmio_write_32(0x07003008UL, 0);
	mmio_write_32(0x07002004UL, 0xFFFFFFFFUL);	/* S_SPC */
	mmio_write_32(0x07002014UL, 0xFFFFFFFFUL);
	mmio_write_32(0x07002024UL, 0xFFFFFFFFUL);
	mmio_setbits_32(0x07010290UL, 0x7);	/* PRCM_SEC_SWITCH */
	mmio_write_32(0x02003F00UL, 0x7);
	mmio_write_32(0x04601020UL, 0xFFFFUL);	/* CE */
	mmio_write_32(0x04024020UL, 0xFFFFUL);	/* DMAC1 */
	mmio_write_32(0x020540E0UL, 0xFFFFFBFFUL);
	mmio_write_32(0x02002800UL, 0x80000002UL);
	mmio_write_32(0x02002850UL, 0x10001UL);
	mmio_write_32(0x03009008UL, 0);	/* TIMER0 */
}

/* returns false when the word is PC-relative and not re-encodable */
static void sunxi_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	unsigned int core = plat_my_core_pos();

	/* The power cycle wiped the EL2 world state; put the baseline back
	 * before this core returns to NS. */
	sunxi_warmboot_fixups();

	/* reference-parity counter re-arm  - the
	 * fresh core's CNTFRQ_EL0 is not guaranteed; re-state it before
	 * any cache/interconnect traffic. */
	sunxi_cntfrq_arm();

	sunxi_apply_cpu_errata();

#ifndef SUNXI_BP243_NO_SRE
	{
		u_register_t scr = read_scr_el3();

		write_icc_sre_el3(read_icc_sre_el3() | ICC_SRE_EN_BIT |
				  ICC_SRE_SRE_BIT);
		isb();
		write_scr_el3(scr | SCR_NS_BIT);
		isb();
		write_icc_sre_el2(read_icc_sre_el2() | ICC_SRE_EN_BIT |
				  ICC_SRE_SRE_BIT);
		isb();
		write_icc_sre_el1(ICC_SRE_SRE_BIT);
		isb();
		write_scr_el3(scr);
		isb();
	}
#endif /* SUNXI_BP243_NO_SRE */

	sunxi_cpu_assist_clear(core);
	sunxi_imd_el3_write(1U);
	/* vendor-parity GIC-600 redistributor power handshake
	 * : if the frame identifies as GIC-600/AE
	 * (GICR_IIDR 0x0200043b / 0x0400043b), run the PWRR-style
	 * handshake on frame+0x24 - poll until bit3^bit2 clears, write 0,
	 * repeat until bit0 reads back 0.  The generic TF-A driver never
	 * touches this register; the vendor monitor does it before its
	 * SGI/PPI defaulting on every core wake. */
	{
		uintptr_t g = SUNXI_GICR_BASE + (uintptr_t)core * (1U << 17);
		uint32_t iidr = mmio_read_32(g + 0x4U);

		if (((iidr & 0xfe000fffU) == 0x0200043bU) ||
		    ((iidr & 0xff000fffU) == 0x0400043bU)) {
			uint32_t v;

			do {
				do {
					v = mmio_read_32(g + 0x24U);
				} while ((((v >> 3) ^ (v >> 2)) & 1U) != 0U);
				mmio_write_32(g + 0x24U, 0U);
				v = mmio_read_32(g + 0x24U);
			} while ((v & 1U) != 0U);
		}
	}
	if (target_state->pwr_domain_state[PLAT_MAX_PWR_LVL] !=
	    PLAT_MAX_OFF_STATE) {
		gicv3_rdistif_init(core);
	}
	gicv3_cpuif_enable(core);
}

static int sunxi_pwr_domain_off_early(const psci_power_state_t *target_state)
{
	VERBOSE("BL31: off_early enter core=%u\n", plat_my_core_pos());
	/*
	 * Take whatever the E902 posted and nobody read: its queue is only
	 * eight words deep and has to have room for the replies the suspend
	 * hand-off will ask for.
	 */
	sunxi_arisc_rx_drain();
	VERBOSE("BL31: off_early leave core=%u\n", plat_my_core_pos());

	return PSCI_E_SUCCESS;
}

static void sunxi_pwr_domain_off(const psci_power_state_t *target_state)
{
	unsigned int core = plat_my_core_pos();

	VERBOSE("BL31: off enter core=%u\n", core);

	/*
	 * Vendor parity: the reference pwr_domain_off (0x8624) only
	 * disables the GIC CPU interface, clears the EL3 IMPDEF word and
	 * programs the CPUIDLE bits.  It returns; the generic PSCI tail
	 * commits AFF_STATE_OFF, releases the power-domain locks and
	 * enters WFI.  Do not commit AFF_STATE_OFF or power the core down
	 * here: doing so lets the PPU remove this core before the generic
	 * bookkeeping runs.
	 */
	gicv3_cpuif_disable(core);
	sunxi_imd_el3_write(0U);
	sunxi_cpu_off();
	VERBOSE("BL31: off wfi core=%u\n", core);
}


/*
 * A system suspend is decided by what the normal world asked for, not by
 * PSCI's state coordination: the generic coordination keeps a domain out
 * of its deepest state while any CPU in it still reports RUN, and the
 * hotplug path can leave such a stale entry behind even though
 * psci_is_last_on_cpu() has already confirmed that this is the only core
 * still running.  The vendor monitor does not consult the negotiated
 * state either.
 */
static bool sunxi_sys_suspending;
static bool sunxi_deep_suspend;

static void sunxi_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	unsigned int core = plat_my_core_pos();
	bool deep = sunxi_sys_suspending ||
		    (target_state->pwr_domain_state[PLAT_MAX_PWR_LVL] ==
		     PLAT_MAX_OFF_STATE);

	sunxi_sys_suspending = false;
	sunxi_deep_suspend = deep;

	/* the hand-off below is a reply the E902 must have room to answer */
	sunxi_arisc_rx_drain();

	/* vendor-parity - the vendor monitor disables this core's
	 * GIC CPU interface at the very top of its suspend path (0x87EC ->
	 * 0x830C) for EVERY suspend, powerdown levels included.  A
	 * quiescent CPU interface is the precondition for the power
	 * controller to grant the per-CPU powerdown and for the
	 * redistributor to raise wake_request when an interrupt targets
	 * the sleeping core. */
	gicv3_cpuif_disable(core);



	if (!deep) {
		return;
	}
	/* NO cm_el1_sysregs_context_save here.  The vendor firmware
	 * (0x87B4) goes straight from the GIC save to the arisc hand-off
	 * and never snapshots the NS EL1 sysregs; the PSCI warmboot path
	 * re-inits the context from the resume entrypoint instead (the
	 * kernel's cpu_resume rebuilds TTBR0/SCTLR itself).  Snapshotting
	 * here (mrs TTBR0/SCTLR + memcpy into DRAM) is extra work the
	 * vendor flow does not do and adds DRAM traffic right before the
	 * ARISC starts tearing down clocks - remove it. */

	/* NOTE: the vendor monitor writes NOTHING to the RTC in its
	 * suspend path .  Writing RTC+0x00 (LOSC_CTRL, keyed
	 * 0x16AA4011) here re-clocks the RTC right after the kernel armed
	 * its alarm and stops the alarm from ever firing - the ARISC
	 * standby then never sees a wake event and the system stays down.
	 * Leave the RTC alone (its boot clock simply ticks fast; the alarm
	 * fires early, which still wakes the system). */

	/* : GIC CPU interface down first (0x830C) */
	gicv3_cpuif_disable(core);
	sunxi_imd_el3_write(0U);

	/*
	 * System-off level : raise the cluster
	 * assist, snapshot the GIC distributor / DRAM capacity / NSI
	 * state, then hand over to the arisc with a single async CPU_OP
	 * (all power states = 3).  The arisc runs its super-standby flow
	 * and resumes the boot core from `resume_entry` (= our
	 * sec_entrypoint, already programmed into the RVBARs).
	 */
	sunxi_cluster_power_assist(0U, core);	/* 0xB3D4 */
	gicv3_rdistif_save(core, &sunxi_rdist_ctx);	/* SGI/PPI ctx */
	gicv3_distif_save(&sunxi_dist_ctx);	/* 0x3D64 */
	sunxi_dma_save();			/* 0x5C0C */
	sunxi_drm_check_region();		/* 0xB440 (0x8854) */

	if ((mmio_read_32(SUNXI_SID_CTRL_FLAG) &
	     SUNXI_SID_FLAG_DRAM_READY) != 0U) {
		sunxi_smc_mcap_a = mmio_read_32(SUNXI_SMC0_MCAP_A);
		sunxi_smc_mcap_b = mmio_read_32(SUNXI_SMC0_MCAP_B);
	}
	sunxi_nsi_word_0 = mmio_read_32(SUNXI_NSI_SUSPEND_W0);
	sunxi_nsi_word_1 = mmio_read_32(SUNXI_NSI_SUSPEND_W1);

	/* the vendor BSP (0x87B4) sends NOTHING to the arisc
	 * here - no loopback ping, no wake-source override.  The kernel
	 * configures the wake source (RTC alarm) itself before the PSCI
	 * call, and the arisc standby honours it.  The old debug override
	 * forced the arisc's own 5s soft-timer as the wake source, which
	 * the vendor flow never does.  Revert to the vendor behaviour. */

	/* the vendor BSP (0x87B4) sends NOTHING to the arisc
	 * here - no loopback ping, no wake-source override, and no log-ring
	 * drain.  The kernel configures the wake source (RTC alarm) itself
	 * before the PSCI call, and the arisc standby honours it.  The old
	 * debug drain read the SRAM ring and wrote a DRAM snapshot, which is
	 * extra DRAM traffic right before the arisc tears the clocks down -
	 * remove it too. */

	/* read IRQ24 enable state BEFORE the CPU_OP - the kernel's
	 * 0x26 wake-source config ran earlier (during late suspend); if
	 * en24=1 here the config worked and something in standby clears
	 * it, if en24=0 the 0x26 never took effect. */
	/*
	 * The hand-off frame is seven words and the E902's receive queue is
	 * eight, so it only fits into a queue this side has just emptied;
	 * arisc_msg_send() drains the other direction and reserves room
	 * before it writes.  CPU_OP is ASYNC - there is no reply to wait
	 * for - so one send is the whole hand-off.
	 */
	(void)sunxi_arisc_cpu_op(read_mpidr_el1(),
				 (uintptr_t)sec_entrypoint_get(),
				 ARISC_POWER_OFF,
				 ARISC_POWER_OFF,
				 ARISC_POWER_OFF);
	/* return to the PSCI framework normally (so the power-domain
	 * locks acquired before suspend are RELEASED), and let the PSCI
	 * suspend path call sunxi_pwr_domain_pwr_down() (implemented below)
	 * which performs the *terminal* WFI.   did the terminal WFI
	 * inside this function, which powered the cluster off correctly but
	 * left the PSCI locks held - the warmboot path then deadlocked
	 * before the ARISC restore notify (flag stopped at 0xf3f3700c). */
	{
		/* EL2 world state (pKVM) - the power cycle resets it;
		 * the vendor monitor replays it from its per-CPU wake init
		 * (0x480048d0 called at 0x48008b48/0x48008e3c/0x48009280). */
		sunxi_el2_backup.hcr = sunxi_read_hcr();
		sunxi_el2_backup.cptr = sunxi_read_cptr();
		sunxi_el2_backup.cnthctl = sunxi_read_cnthctl();
		sunxi_el2_backup.cntvoff = sunxi_read_cntvoff();
		sunxi_el2_backup.vpidr = sunxi_read_vpidr();
		sunxi_el2_backup.vmpidr = sunxi_read_vmpidr();
		sunxi_el2_backup.vttbr = sunxi_read_vttbr();
		sunxi_el2_backup.mdcr = sunxi_read_mdcr();
		sunxi_el2_backup.hstr = sunxi_read_hstr();
		sunxi_el2_backup.cnthp_ctl = sunxi_read_cnthp_ctl();
		sunxi_el2_valid = true;
	}
	/* vendor-parity tail - the vendor monitor never writes the
	 * CPUIDLE POWER_ON bit (0x07050220) nor arms the R-domain watchdog
	 * (0x07021000) on the suspend path (verified against the vendor
	 * image: zero references to both registers in the whole binary).
	 * Both were open-firmware additions: the POWER_ON clear may disturb
	 * the R-domain/PPU state that the kernel's early dpm_resume_noirq
	 * walk (first touch = R_I2C twi-7085000) has to go through, and the
	 * ARISC manages its own recovery watchdog anyway.  Removed so the
	 * suspend tail matches the vendor flow exactly. */
	dsb();
	isb();
}

/* terminal power-down WFI, invoked by the PSCI suspend path AFTER
 * the power-domain locks have been released (psci_suspend.c line 240).
 * This mirrors the vendor BSP's psci_power_down_wfi(): an
 * unrecoverable wfi that lets the PPU power the core/cluster off.  The
 * ARISC then resumes the boot core from the RVBAR when the wake source
 * fires. */
static void sunxi_pwr_domain_pwr_down(const psci_power_state_t *target_state)
{
	/*
	 * park the core only for a system-level suspend.  A
	 * CPU/cluster-level suspend (the kernel's cpuidle CPU_SLEEP_0 /
	 * CLUSTER_SLEEP_0 -> PSCI CPU_SUSPEND powerdown) is not handed to
	 * the ARISC: nobody would ever cut this core's power or resume it
	 * from the RVBAR, so trapping it here wedges the core in EL3
	 * forever (no timers, no IPI service - every subsequent RCU grace
	 * period then stalls).  Return to the PSCI framework, which
	 * executes its own WFI and unwinds the abandoned powerdown in
	 * place, resuming the kernel in-line.
	 */
	if (!sunxi_deep_suspend) {
		return;
	}

	dsb();
	isb();
	for (;;)
		__asm__ volatile ("wfi" ::: "memory");
	/* not reached */
}


static void sunxi_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	unsigned int core = plat_my_core_pos();

	sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE1000000U);
	/* The standby power cycle wiped the EL2 world state. */
	sunxi_warmboot_fixups();
	sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE2000000U);


	/* no console prints in the resume window - UART0 clock may
	 * be gated and 16550 putc spins on THRE; the RTC tags below are the
	 * only channel until the kernel re-opens the console. */
	/* warm step timer - free-running count at each milestone */
	if (sunxi_deep_suspend) {
		/* restore the full hardware state, mirroring vendor
		 * 0x88B0 exactly (GIC re-init + dist ctx restore -> DMA ->
		 * notify -> NSI -> CCU -> SYSCTRL+8 -> SMC -> system_restore
		 * -> assist clear).   skipped all of this and the kernel
		 * could not survive - GIC restore is mandatory. */
		/* 0x3D70: full distributor restore (init + saved SPIs) */
		sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE3000000U);
		gicv3_distif_init_restore(&sunxi_dist_ctx);
		sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE4000000U);
		/* restore the SGI/PPI context instead of the old
		 * gicv3_rdistif_init(), which wiped the tick PPI 27 and
		 * all IPIs of this core (see sunxi_rdist_ctx). */
		gicv3_rdistif_init_restore(core, &sunxi_rdist_ctx);
		gicv3_cpuif_enable(core);
		sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE5000000U);

		sunxi_dma_restore();			/* 0x5BBC */
		sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE6000000U);

		/* AP resume complete: notify the arisc (0x1A3C); the reply
		 * feeds the vendor SIP query FID 0x23. */
		sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE7000000U);
		sunxi_standby_result =
			(uint32_t)sunxi_arisc_standby_restore_notify();
		sunxi_s3_trace(SUNXI_RTC_S3_TRACE_REG, 0xE8000000U);

		/* NSI words (0x8934) */
		mmio_write_32(SUNXI_NSI_SUSPEND_W0, sunxi_nsi_word_0);
		mmio_write_32(SUNXI_NSI_SUSPEND_W1, sunxi_nsi_word_1);

		/* 0xC8A0 CCU/PLL restore  */
		sunxi_ccu_restore();

		/*
		 * The UART0 gate/reset is restored by
		 * sunxi_system_restore() below, exactly like the vendor
		 * monitor (0xC794 writes 0x02002850 = 0x10001 and
		 * 0x02002800 = 0x80000002).  Do not write 0x02002E00
		 * here: that is the PLL-gate array which the CCU restore
		 * only reads to find the active PLL, and the vendor never
		 * writes it.
		 */
		mmio_write_32(SUNXI_SYSCTRL_BASE + 0x8U, 0U);	/* 0x8954 */
		sunxi_smc_cap_restore();		/* 0xB3F4 */
		sunxi_system_restore();			/* 0xC794 */
		sunxi_cpu_assist_clear(core);		/* 0xC600 */
		/* NOTE the recovery watchdog is no longer armed in
		 * suspend (see suspend path); nothing to disarm here.  The
		 * suspend flag stays set until
		 * sunxi_pwr_domain_on_finish() below has run. */
	}

	/*
	 * an idle CPU/cluster-level suspend (state[2] != OFF) wakes
	 * up in-line with its full context intact - the platform tore
	 * nothing down for it.  Skip the on-finish reset tail (CPUIF/SRE
	 * re-init + GICR defaultization) so the kernel's live per-CPU GIC
	 * configuration (tick PPI, IPIs, IRQ enables) survives the nap;
	 * the kernel's CPU_PM exit path only restores GIC system
	 * registers, not per-interrupt enables.  The system-suspend
	 * (state[2]==OFF) resume and the CPU_ON bring-up still run the
	 * full tail.
	 */
	if (!sunxi_deep_suspend) {
		uintptr_t g = SUNXI_GICR_BASE + (uintptr_t)core * (1U << 17);

		/* vendor-parity light resume tail (its common
		 * tail's non-system branch, 0x8368): GIC-600 redistributor
		 * power handshake (0x6B28) -> full CPU interface re-enable
		 * (0x6044) -> private-interrupt defaultization (0x3D90).
		 * The kernel's own PM exit only restores GIC system
		 * registers, so the firmware must bring the interface back
		 * after its suspend-side gicv3_cpuif_disable(). */
		{
			uint32_t iidr = mmio_read_32(g + 0x4U);

			if (((iidr & 0xfe000fffU) == 0x0200043bU) ||
			    ((iidr & 0xff000fffU) == 0x0400043bU)) {
				uint32_t v;

				do {
					do {
						v = mmio_read_32(g + 0x24U);
					} while ((((v >> 3) ^ (v >> 2)) & 1U) != 0U);
					mmio_write_32(g + 0x24U, 0U);
					v = mmio_read_32(g + 0x24U);
				} while ((v & 1U) != 0U);
			}
		}
		gicv3_cpuif_enable(core);
		isb();
		/* private-interrupt defaultization (0x3D90) - the
		 * reference monitor normalizes the redistributor's SGI/PPI
		 * registers after every resume before handing the core back
		 * to the kernel. */
		mmio_write_32(g + 0x10080U, 0xffffffffU);	/* ICENABLER0 */
		mmio_write_32(g + 0x10D00U, 0U);		/* IGRPMODR0 */
		for (uint32_t i = 0U; i < 32U; i += 4U) {
			mmio_write_32(g + 0x10400U + i, 0x80808080U);
		}
		dsb();
		return;
	}

	sunxi_deep_suspend = false;
	/* Marker only - a console print here can itself wedge the resume
	 * (UART0's clock/state is not reliable in this window, 16550 putc
	 * spins on THRE).  E9 in 0x07090110, read out by the ARISC's
	 * witness dump, proves BL31 reached the end of its restore tail;
	 * combined with E1..E8 in SUNXI_RTC_S3_TRACE_REG it brackets any
	 * resume hang to either BL31's tail or the return-to-NS/kernel
	 * side.  The two registers are otherwise unused RTC GPRs. */
	sunxi_s3_trace(SUNXI_RTC_S3_STAGE_REG, 0xE9000000U);
	sunxi_pwr_domain_on_finish(target_state);

	/* EA: the whole suspend-finish tail (deep restore + on_finish's
	 * GIC/EL2 work) is done; what remains is the return to the PSCI
	 * framework and the ERET to the kernel's resume entry. */
	sunxi_s3_trace(SUNXI_RTC_S3_STAGE_REG, 0xEA000000U);
}

static int sunxi_validate_power_state(unsigned int power_state,
				      psci_power_state_t *req_state)
{
	unsigned int pwr_lvl = psci_get_pstate_pwrlvl(power_state);
	unsigned int type = (power_state >> 16) & 1U;
	unsigned int i;

	/* : level 3 invalid; low 16 bits must be zero;
	 * bit16 selects standby (state 1, level <= 1) vs powerdown
	 * (state 2); slots above the requested level are cleared. */
	if (pwr_lvl >= PLAT_MAX_PWR_LVL + 1U)
		return PSCI_E_INVALID_PARAMS;

	if ((power_state & 0xffffU) != 0U)
		return PSCI_E_INVALID_PARAMS;

	if (type == 0U) {	/* standby */
		if (pwr_lvl >= 2U)
			return PSCI_E_INVALID_PARAMS;

		for (i = 0U; i <= pwr_lvl; i++)
			req_state->pwr_domain_state[i] = PLAT_MAX_RET_STATE;
	} else {		/* powerdown */
		for (i = 0U; i <= PLAT_MAX_PWR_LVL; i++)
			req_state->pwr_domain_state[i] = 0U;
		req_state->pwr_domain_state[0] = PLAT_MAX_RET_STATE;
	}

	for (i = pwr_lvl + 1U; i <= PLAT_MAX_PWR_LVL; i++)
		req_state->pwr_domain_state[i] = 0U;

	return PSCI_E_SUCCESS;
}

static void sunxi_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	unsigned int i;

	/*
	 * Leave the UART0/PLL gate alone here: the kernel owns the console
	 * clock and the vendor monitor writes nothing before the hand-off.
	 * sunxi_system_restore() restores the vendor's UART0 gate word
	 * (0x02002850) on resume.
	 */
	sunxi_sys_suspending = true;

	for (i = 0; i <= PLAT_MAX_PWR_LVL; i++)
		req_state->pwr_domain_state[i] = PLAT_MAX_OFF_STATE;
}

static void sunxi_cpu_standby(plat_local_state_t cpu_state)
{
	uint32_t scr = read_scr_el3();
	unsigned int core = plat_my_core_pos();

	(void)core;
	(void)cpu_state;
	assert(cpu_state == PLAT_MAX_RET_STATE);

	/* route FIQ/IRQ to EL3 around the wfi, as  does */
	write_scr_el3(scr | SCR_IRQ_BIT | SCR_FIQ_BIT);
	isb();
	dsb();
	wfi();
	write_scr_el3(scr);
	isb();
}

static void __dead2 sunxi_system_off(void)
{
	/* SYS_OP is sent unconditionally: the
	 * E902 may already be running (boot0 loads/starts it), so do not
	 * gate on our own FF10 load state.  The transport waits have a
	 * finite timeout, so a missing arisc cannot hang the core. */
	(void)sunxi_arisc_sys_op(ARISC_SYS_SHUTDOWN);

	/* vendor string (0x10FFA; "aw" is Allwinner shorthand) */
	VERBOSE("aw System Off: operation not handled.\n");
	for (;;) {
		wfi();
	}
}

static void __dead2 sunxi_system_reset(void)
{
	/* Forensics: stamp RTC+0x11C so the next cold boot can tell a
	 * kernel-initiated PSCI SYSTEM_RESET (panic reboot path) apart
	 * from the R_WDOG recovery reset (which leaves no stamp). */

	/* Unconditional SYS_OP(1), same as  */
	(void)sunxi_arisc_sys_op(ARISC_SYS_REBOOT);

	/* WDT0 fallback: with ARISC absent (no-arisc build) the SYS_OP
	 * message goes unanswered and the old code dead-loops forever.
	 * WDT0 @0x02050000 with the sequence the EDK2 SunxiWdtResetSystemLib
	 * validates on this board (cfg@0x14 key|reset=SYS, mode@0x18
	 * key|enable=0.5s, ctl@0x10 restart key 0xA57<<1): hardware reset
	 * within ~0.5 s, no co-processor involved. */
	udelay(10000);	/* give a healthy ARISC the first move (PMIC reset) */
	mmio_write_32(0x02050014UL, 0x16AA0001U);	/* CFG: key | reset=SYS */
	mmio_write_32(0x02050018UL, 0x16AA0001U);	/* MODE: key | enable */
	mmio_write_32(0x02050010UL, 0x0A57U << 1 | 1U);	/* CTL: restart */

	/* vendor string (0x10FAE) */
	VERBOSE("aw System Reset: operation not handled.\n");
	for (;;) {
		wfi();
	}
}

static const plat_psci_ops_t sunxi_psci_ops = {
	.cpu_standby			= sunxi_cpu_standby,
	.pwr_domain_on			= sunxi_pwr_domain_on,
	.pwr_domain_on_finish		= sunxi_pwr_domain_on_finish,
	.pwr_domain_off_early		= sunxi_pwr_domain_off_early,
	.pwr_domain_off			= sunxi_pwr_domain_off,
	.pwr_domain_suspend		= sunxi_pwr_domain_suspend,
	.pwr_domain_pwr_down		= sunxi_pwr_domain_pwr_down,
	.pwr_domain_suspend_finish	= sunxi_pwr_domain_suspend_finish,
	.validate_power_state		= sunxi_validate_power_state,
	.get_sys_suspend_power_state	= sunxi_get_sys_suspend_power_state,
	.system_off			= sunxi_system_off,
	.system_reset			= sunxi_system_reset,
	.validate_ns_entrypoint		= NULL,
};

static const uint32_t sunxi_ppu_blocks[] = {
	0x07051000U, 0x07052000U, 0x07053000U, 0x07054000U, 0x07055000U,
	0x07057000U, 0x07058000U, 0x0705A000U, 0x0705B000U, 0x0705D000U,
	0x0705E000U,
};

int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const struct plat_psci_ops **psci_ops)
{
	unsigned int i;

	sunxi_sec_entrypoint = sec_entrypoint;

	/* : program core 0 only (AA64NAA32 + RVBAR; the
	 * other cores get theirs on PSCI CPU_ON) ... */
	mmio_setbits_32(SUNXI_CPU_SUBSYS_BASE + 0x1000U, BIT(0));
	mmio_write_32(SUNXI_CPU_SUBSYS_BASE + 0x1004U,
		      (uint32_t)(sec_entrypoint & 0xffffffffU));
	mmio_write_32(SUNXI_CPU_SUBSYS_BASE + 0x1008U, 0U);

	/* ... then clear the low PPU nibble of the vendor's PPU blocks
	 * (0x4F94).  The list is not a 0x1000 stride: it skips
	 * 0x07056000/0x07059000 and continues past 0x0705F000.  The GPU
	 * blocks 0x07060000/0x07061000 are left alone because clearing
	 * them on this board powers down the GPU domain and the AP console
	 * stops at ~1.9 s in pdtest, while every CPU still reaches 0x108. */
	for (i = 0U; i < ARRAY_SIZE(sunxi_ppu_blocks); i++)
		mmio_clrbits_32(sunxi_ppu_blocks[i], 0xfU);

	*psci_ops = &sunxi_psci_ops;
	return 0;
}
