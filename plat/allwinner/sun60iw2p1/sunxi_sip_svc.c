/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Allwinner vendor SIP service (OEN 0) and SMCCC forwarding.
 */
#include <assert.h>
#include <string.h>

#include <platform_def.h>

#include <arch.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/ep_info.h>
#include <common/runtime_svc.h>
#include <arch_helpers.h>
#include <arch/aarch64/smccc_helpers.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <lib/utils.h>
#include <lib/smccc.h>
#include <plat/common/platform.h>
#include <services/arm_arch_svc.h>

#include <sunxi_def.h>
#include <sunxi_private.h>

#define SUNXI_SIP_FUNC_MASK		0xffffU

/* The reference handler answers different FID sets depending on the
 * SMC32/SMC64 form (bit 30), so gate each vendor command accordingly. */
static inline bool sunxi_is_smc64(uint32_t smc_fid)
{
	return (((smc_fid >> FUNCID_CC_SHIFT) & 1U) != 0U);
}

/* SMCCC arch calls that must keep working despite the OEN-0 takeover */
#define SUNXI_SMCCC_FID(func)		((func) | (SMC_64 << FUNCID_CC_SHIFT))

static bool is_standard_smccc(uint32_t smc_fid)
{
	/* both yielding and fastcall, 32/64 bit, func 0..3 */
	return ((smc_fid & SUNXI_SIP_FUNC_MASK) <= 0x3U);
}

/* vendor kernel ABI FIDs (bsp/drivers/smc/sunxi-smc.c) */
#define SUNXI_SIP_FID_EFUSE_READ	0xFE00U
#define SUNXI_SIP_FID_EFUSE_WRITE	0xFE01U
#define SUNXI_SIP_FID_READ_SEC_REG	0xFF05U
#define SUNXI_SIP_FID_WRITE_SEC_REG	0xFF06U
#define SUNXI_SIP_FID_EFUSE_READ_EXTRA	0xFF07U
#define SUNXI_SIP_FID_REBOOT_EFEX	0xFF18U
#define SUNXI_SIP_FID_MON_PARAM		0xFF41U	/* vendor CPU_ON (psci_cpu_on) */

/* 0xFF14 is an unsupported (-1) stub in the vendor BSP; no monitor parameter
 * state is kept. */
/* arch_helpers exports no ACTLR_EL3 accessor, so write it inline. */
static inline void sunxi_write_actlr_el3(uint64_t v)
{
	__asm__ volatile("msr actlr_el3, %0" : : "r"(v) : "memory");
}
static inline uint64_t sunxi_read_scr_el3(void)
{
	uint64_t v;
	__asm__ volatile("mrs %0, scr_el3" : "=r"(v));
	return v;
}
static inline void sunxi_write_scr_el3(uint64_t v)
{
	__asm__ volatile("msr scr_el3, %0" : : "r"(v) : "memory");
}
#define SUNXI_SIP_FID_CPU_HW_STATE	0xFF42U	/* per-core PPU state query  */
#define SUNXI_SIP_FID_ACTLR_SMPEN	0xFF50U	/* ACTLR_EL3 = 0xC00         */
#define SUNXI_SIP_FID_COPY_ARISC_PARAS	0xFF10U
#define SUNXI_SIP_FID_GET_TEEADDR	0xFF11U
#define SUNXI_SIP_FID_GET_DRM_INFO	0xFF0FU
#define SUNXI_SIP_FID_INFORM_FDT	0xFF19U

static uint32_t smccc_version_word = MAKE_SMCCC_VERSION(1, 2);

static uintptr_t sunxi_std_smccc(uint32_t smc_fid, u_register_t x1,
				 void *handle)
{
	switch (smc_fid & SUNXI_SIP_FUNC_MASK) {
	case 0x0:	/* SMCCC_VERSION */
		SMC_RET1(handle, smccc_version_word);
	case 0x1:	/* SMCCC_ARCH_FEATURES, mirroring  */
		if (x1 == 0x80007FFFU) {		/* invalid par, -2 */
			SMC_RET1(handle, (u_register_t)-2);
		} else if (x1 < 0x80000000ULL) {	/* the vendor BSP returns 1 */
			SMC_RET1(handle, 1U);
		} else if (x1 == 0x80000000U || x1 == 0x80000001U) {
			SMC_RET1(handle, SMC_ARCH_CALL_SUCCESS);
		} else if (x1 == 0x80008000U) {		/* ARCH_WORKAROUND_1 */
			/* the vendor BSP: check_wa_cve_2017_5715() (0x29A0)
			 * reads cpu_ops.extra1, which is NULL for both
			 * the A55 (0x410FD050) and A76 (0x410FD0B0)
			 * entries, so it returns ERRATA_NOT_APPLIES and
			 * the SMC answers 1 = "unaffected" (SMCCC /
			 * Linux SMCCC_ARCH_WORKAROUND_RET_UNAFFECTED).
			 * This build likewise compiles no WA1 hooks. */
			SMC_RET1(handle, 1U);
		} else {				/* 0x80000002..: -1 */
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		}
	case 0x2:	/* SMCCC_ARCH_SOC_ID: x1==0/1 -> -1, else -3 */
		if (x1 <= 1U) {
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		} else {
			SMC_RET1(handle, (u_register_t)-3);
		}
	default:
		SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
	}
}

/* The live standby-restore result lives in sunxi_pm.c
 * (sunxi_standby_result, filled from the 0x11 notify reply). */

/* FID 0x8000FF10 originally documented as a copy-ARISC-params helper is
 * actually the arisc firmware load request ;
 * the DRAM-copy helper was removed accordingly. */

static inline uint64_t sunxi_cpu_imd_read(unsigned int idx)
{
	uint64_t v = 0U;

	switch (idx) {
	case 0U: __asm__ volatile("mrs %0, S3_3_C15_C9_0" : "=r"(v)); break;
	case 1U: __asm__ volatile("mrs %0, S3_3_C15_C9_1" : "=r"(v)); break;
	case 2U: __asm__ volatile("mrs %0, S3_3_C15_C9_2" : "=r"(v)); break;
	case 3U: __asm__ volatile("mrs %0, S3_3_C15_C9_3" : "=r"(v)); break;
	case 4U: __asm__ volatile("mrs %0, S3_3_C15_C9_4" : "=r"(v)); break;
	case 5U: __asm__ volatile("mrs %0, S3_3_C15_C9_5" : "=r"(v)); break;
	case 6U: __asm__ volatile("mrs %0, S3_3_C15_C9_6" : "=r"(v)); break;
	case 7U: __asm__ volatile("mrs %0, S3_3_C15_C9_7" : "=r"(v)); break;
	default: break;
	}
	return v;
}

static inline void sunxi_cpu_imd_write(unsigned int idx, uint64_t v)
{
	switch (idx) {
	case 0U: __asm__ volatile("msr S3_3_C15_C9_0, %0" :: "r"(v)); break;
	case 1U: __asm__ volatile("msr S3_3_C15_C9_1, %0" :: "r"(v)); break;
	case 2U: __asm__ volatile("msr S3_3_C15_C9_2, %0" :: "r"(v)); break;
	case 3U: __asm__ volatile("msr S3_3_C15_C9_3, %0" :: "r"(v)); break;
	case 4U: __asm__ volatile("msr S3_3_C15_C9_4, %0" :: "r"(v)); break;
	case 5U: __asm__ volatile("msr S3_3_C15_C9_5, %0" :: "r"(v)); break;
	case 6U: __asm__ volatile("msr S3_3_C15_C9_6, %0" :: "r"(v)); break;
	case 7U: __asm__ volatile("msr S3_3_C15_C9_7, %0" :: "r"(v)); break;
	default: break;
	}
}

static int sunxi_sip_cpu_probe(uintptr_t o0, uintptr_t o1, uintptr_t o2)
{
	bool is_a76 = (((read_midr_el1() >> 4) & 0xfffUL) == 0xD0BUL);

	mmio_write_64(o0, 0U);
	mmio_write_64(o1, 0U);
	mmio_write_64(o2, 0U);
	if (!is_a76)
		return 0;

	write_cptr_el2(read_cptr_el2() & ~BIT(30));
	write_cptr_el3(read_cptr_el3() & ~BIT(30));

	mmio_write_64(o0, sunxi_cpu_imd_read(0U));
	mmio_write_64(o1, sunxi_cpu_imd_read(2U));
	mmio_write_64(o2, sunxi_cpu_imd_read(3U));
	sunxi_cpu_imd_write(0U, 0U);
	sunxi_cpu_imd_write(2U, 0U);
	sunxi_cpu_imd_write(3U, 0U);

	__asm__ volatile("msr S3_3_C15_C9_6, %0" :: "r"(31UL));

	return 0;
}

/*
 * : FF04/FF40 "next image" debug service.  Builds an
 * entry_point_info for the given pc and re-initialises the non-secure
 * context so that the SMC return path (el3_exit) resumes at the new
 * entry point instead of the SMC caller:
 *   x3 == 0: AArch32 HYP (spsr 0x1d3), args[2] = x2
 *   x3 == 1: AArch64, spsr = mode ? 0x3c9 (EL2h) : 0x3c5 (EL1h),
 *            args[0] = x2
 *   other  : zeroed EP, faithfully to 0x8b5c (b.ne 0x8b04)
 */
static void sunxi_sip_next_image(u_register_t x1, u_register_t x2,
				 u_register_t x3, uint32_t mode)
{
	entry_point_info_t ep;

	memset(&ep, 0, sizeof(ep));
	SET_SECURITY_STATE(ep.h.attr, NON_SECURE);	/* [base+4] |= 1 */

	if ((uint32_t)x3 == 0U) {
		ep.pc = (uint32_t)x1;
		ep.spsr = 0x1D3U;	/* AArch32 HYP, F/I masked */
		ep.args.arg2 = (uint32_t)x2;	/* EP+0x28 (0x8aec) */
	} else if ((uint32_t)x3 == 1U) {
		ep.pc = (uint32_t)x1;
		ep.spsr = (mode != 0U) ? 0x3C9U : 0x3C5U;
		ep.args.arg0 = (uint32_t)x2;	/* EP+0x18 (0x8b7c) */
	}

	NOTICE("BL3-1: Next image address = 0x%llx\n",
	       (unsigned long long)ep.pc);
	NOTICE("BL3-1: Next image spsr = 0x%x\n", ep.spsr);

	/* 0x8b24-0x8b30: caches off for the next image */
	write_sctlr_el1(read_sctlr_el1() &
			~((u_register_t)SCTLR_M_BIT | (u_register_t)SCTLR_C_BIT));
	/* 0x8b38: full data-cache clean+invalidate (dcsw_op_all DCCISW) */
	dcsw_op_all(DCCISW);

	/* 0x8b40/0x8b48: context setup + programmed EL3 exit */
	cm_init_my_context(&ep);
	cm_prepare_el3_exit_ns();
}

static u_register_t sunxi_sip_handler(uint32_t smc_fid, u_register_t x1,
				      u_register_t x2, u_register_t x3,
				      void *handle)
{
	uint32_t cmd = smc_fid & SUNXI_SIP_FUNC_MASK;
	const uint32_t paras[3] = { (uint32_t)x1, (uint32_t)x2, (uint32_t)x3 };

	switch (cmd) {
	case 0x23U:	/* last standby restore result  */
		*(uint32_t *)(uintptr_t)x1 = sunxi_standby_result;
		SMC_RET1(handle, 0U);
	case SUNXI_SIP_FID_EFUSE_READ:
		SMC_RET1(handle, (u_register_t)sunxi_efuse_read(x1, x2));
	case SUNXI_SIP_FID_EFUSE_READ_EXTRA:	/* SMC64 only (FF07) */
		if (!sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		/* : (x1=key name "rgn0", x2=out, x3=len==4) */
		SMC_RET1(handle, (u_register_t)sunxi_efuse_read_socid_xored(x1, x2, x3));
	case SUNXI_SIP_FID_EFUSE_WRITE:
		SMC_RET1(handle, (u_register_t)sunxi_efuse_write(x1, x2));
	case 0xFF20U:	/* SMC64 only; A76 CPU probe  */
		if (!sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		SMC_RET1(handle, (u_register_t)sunxi_sip_cpu_probe(x1, x2, x3));
	case SUNXI_SIP_FID_READ_SEC_REG: {	/* SMC32 and SMC64 (FF05) */
		uint32_t val = 0;
		int rc = sunxi_read_sec_reg(x1, &val);

		/* : not whitelisted -> 0 */
		if (rc == 0)
			SMC_RET1(handle, 0U);
		SMC_RET1(handle, (u_register_t)val);
	}
	case SUNXI_SIP_FID_WRITE_SEC_REG:	/* SMC64 only (FF06) */
		if (!sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		SMC_RET1(handle, (u_register_t)sunxi_write_sec_reg(x1, (uint32_t)x2));
	case SUNXI_SIP_FID_COPY_ARISC_PARAS:	/* SMC32/SMC64 (0x4000FF10/0x8000FF10) */
		/* Accept both calling conventions: EDK2's AriscStartupDxe uses
		 * the SMC64 form (0x8000FF10) while the vendor BSP
		 * answers the SMC32 form - the FID is a 32-bit arg-only
		 * load request, so the CC bit does not matter. */
		NOTICE("BL31: FF10 arisc load arg=0x%lx\n", (unsigned long)x1);
		/* : FID 0x8000FF10 -> 0xAF64 = arisc firmware
		 * load; x1 is the boot argument. */
		SMC_RET1(handle, (u_register_t)sunxi_arisc_init((uint32_t)x1));
	case 0x25U:	/* CLEAR_WAKEUP_SRC_REQ: 1 word  */
	case 0x26U:	/* SET_WAKEUP_SRC_REQ:   1 word (0x21C4) */
	case 0x60U:	/* SET_DEBUG_LEVEL_REQ:  1 word (0x20A8) */
	{
		int result;

		/*
		 * The E902 can only answer this frame if its reply fits in
		 * the AP's eight-word receive queue, and nothing on the AP
		 * side empties that queue for it.  Take what is stuck before
		 * asking for another answer - a full queue here is exactly
		 * what strands the suspend hand-off.
		 */
		sunxi_arisc_rx_drain();
		result = sunxi_arisc_msg_send(cmd, paras, 1U);
		/*
		 * No console output here: these SIPs arrive from the
		 * kernel's wake arm/disarm paths after the UART0 clock has
		 * been gated.  A 16550 putc would spin on THRE and hang the
		 * AP inside BL31.
		 */
		SMC_RET1(handle, (u_register_t)result);
	}
	case 0x61U: {	/* MESSAGE_LOOPBACK: fixed 11-word frame (0x1FDC) */
		static const uint32_t loopback[11] = { 0 };

		SMC_RET1(handle, (u_register_t)sunxi_arisc_msg_send(cmd, loopback, 11U));
	}
	case 0x62U:	/* SET_UART_BAUDRATE */
	case 0x96U:	/* SET_DDRFREQ */
		SMC_RET1(handle, (u_register_t)sunxi_arisc_msg_send(cmd, paras, 1U));
	case 0x64U:	/* SET_DRAM_CRC_PARAS: enable, src, len */
		SMC_RET1(handle, (u_register_t)sunxi_arisc_msg_send(cmd, paras, 3U));
	case 0xFF14U:	/* : unsupported (-1) */
	case 0x24U:	/* vendor: -1 in both forms */
	case 0x27U:	/* vendor stub 0x1A88 (-1) */
	case 0x28U:	/* vendor: -1 */
	case 0x29U:	/* vendor stub 0x201C (-1) */
	case 0x63U:	/* vendor: -1 */
	case 0xA0U:	/* vendor stub 0x3F7C (-1, SMC64) */
	case 0xFF16U:	/* not dispatched in the vendor BSP */
		SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
	case 0xFF04U:	/* next-image debug, AArch32 HYP entry */
	case 0xFF40U:	/* next-image debug, AArch64 entry */
		if (sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		NOTICE("BL31: FF%02x next-image x1=0x%lx x2=0x%lx x3=0x%lx\n",
		       cmd & 0xffU, (unsigned long)x1, (unsigned long)x2,
		       (unsigned long)x3);
		sunxi_sip_next_image(x1, x2, x3,
				     (cmd == 0xFF40U) ? 1U : 0U);
		/* the BL33 context was re-initialised: the SMC return path
		 * resumes at the new entry point instead of the caller */
		SMC_RET0(handle);
	case 0xFF60U: {	/* ARM_SVC_SUNXI_DRM_OPS command 1 (0x2410) */
		uintptr_t desc;
		uint32_t drm_cmd;

		if (sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);

		desc = (uintptr_t)(uint32_t)x1 |
		       ((uintptr_t)(uint32_t)x2 << 32);
		/* Registered safety deviation: keep the descriptor inside
		 * DRAM so a stray pointer cannot abort EL3 (the reference
		 * firmware dereferences it unvalidated). */
		if (!sunxi_drm_ptr_ok(desc, 32U)) {
			NOTICE("DRM: descriptor 0x%lx out of DRAM\n",
			       (unsigned long)desc);
			SMC_RET1(handle, (u_register_t)-1);
		}
		drm_cmd = *(const volatile uint32_t *)desc;
		if (drm_cmd == 1U) {
			u_register_t a0 = *(const volatile uint64_t *)(desc + 8U);
			u_register_t a1 = *(const volatile uint64_t *)(desc + 16U);
			u_register_t a2 = *(const volatile uint64_t *)(desc + 24U);

			SMC_RET1(handle, (u_register_t)(int64_t)(int32_t)
				 sunxi_drm_copy_from_unsafe(a0, a1, a2));
		}
		NOTICE("%s(): Unknown cmd 0x%x for ARM_SVC_SUNXI_DRM_OPS\n",
		       "sunxi_smc_handler_compat", drm_cmd);
		SMC_RET1(handle, (u_register_t)-1);
	}
	case 0xFF41U: {	/* vendor CPU_ON: x1 = target MPIDR, x2 = entry PC,
			 * x3 = context_id.  Same contract as psci_cpu_on():
			 * bad MPIDR -> -2, unaligned entry -> -9, already on
			 * -> -4, powering on -> -5, success -> 0.  The SPSR
			 * comes from psci_get_ns_ep_info(), driven by SCR
			 * (SCR.HCE -> EL2h, else EL1h or AArch32). */
		int core;

		/* SMC32 only (0x8000FF41) */
		if (sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);

		/* An entry address with bit 31 set is a 64-bit address,
		 * i.e. an AArch64 entry: set SCR_EL3.RW so the SPSR takes
		 * the AArch64 form. */
		if ((x2 >> 31) != 0U) {
			uint64_t scr = sunxi_read_scr_el3();

			if ((scr & BIT(10)) == 0U)
				sunxi_write_scr_el3(scr | BIT(10));
		}

		/* Reject an invalid MPIDR before PSCI looks up per-CPU
		 * data for an out-of-range core index. */
		core = plat_core_pos_by_mpidr(x1);
		if (core < 0)
			SMC_RET1(handle, (u_register_t)(uint32_t)PSCI_E_INVALID_PARAMS);

		SMC_RET1(handle, (u_register_t)(uint32_t)psci_cpu_on(x1,
					       (uintptr_t)x2, x3));
	}
	case 0xFF42U: {	/* CPU PPU state query  */
		int core = plat_core_pos_by_mpidr(x1);

		/* SMC32 only (0x8000FF42) */
		if (sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);

		SMC_RET1(handle, (u_register_t)((core >= 0) ?
			(sunxi_cpu_is_powered((unsigned int)core) ? 1U : 0U) : 0U));
	}
	case 0xFF50U:	/* SMC32 only; ACTLR_EL3 = 0xC00  */
		if (sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		sunxi_write_actlr_el3(0xC00U);
		SMC_RET1(handle, 0U);
	case 0xFF18U:	/* reboot into USB efex flashing mode */
		mmio_write_32(SUNXI_RTC_BASE + 0x118, 0x5A);
		(void)sunxi_arisc_sys_op(ARISC_SYS_REBOOT);
		SMC_RET1(handle, 0U);
	case 0xFF15U:	/* SMC32 only; U-Boot shutdown request */
		if (sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
		(void)sunxi_arisc_sys_op(ARISC_SYS_UBOOT_SHUTDOWN);
		SMC_RET1(handle, 0U);
	default:
		NOTICE("SIP: unhandled SIP call 0x%x\n", smc_fid);
		SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);
	}
}

static uintptr_t sunxi_sip_smc_handler(uint32_t smc_fid,
				       u_register_t x1,
				       u_register_t x2,
				       u_register_t x3,
				       u_register_t x4,
				       void *cookie, void *handle,
				       u_register_t flags)
{
	(void)x4;
	(void)cookie;
	(void)flags;

	if (is_standard_smccc(smc_fid)) {
		/* the reference handler only answers the SMC32 forms */
		if (sunxi_is_smc64(smc_fid))
			SMC_RET1(handle, SMC_ARCH_CALL_NOT_SUPPORTED);

		return sunxi_std_smccc(smc_fid, x1, handle);
	}

	return sunxi_sip_handler(smc_fid, x1, x2, x3, handle);
}

/*
 * OEN-0 takeover, matching the vendor BSP: runtime_svc_init lets
 * arm_arch_svc (registered after this descriptor) own the OEN-0 index, so
 * arm_arch_svc forwards every call here.  This is what makes the standard
 * SMCCC answers above (SMCCC 1.2 word, WA1 = 1) and all vendor FIDs
 * reachable at runtime.
 */
bool plat_arch_svc_override(uint32_t smc_fid, u_register_t x1,
			     u_register_t x2, u_register_t x3, void *handle)
{
	(void)sunxi_sip_smc_handler(smc_fid, x1, x2, x3, 0U, NULL, handle, 0U);
	return true;
}

/*
 * The SMCCC service owns OEN 0 and answers the standard calls itself, so
 * the vendor FIDs come in through the plat_arch_svc_override() hook
 * (see services/arm_arch_svc) rather than through a second descriptor:
 * a duplicate OEN-0 descriptor would only be reached if the link order
 * changed.
 */
