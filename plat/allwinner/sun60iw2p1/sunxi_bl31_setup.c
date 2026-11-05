/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BL31 platform setup for the A733, following the flow recovered from
 * the vendor BSP (sunxi_bl31_setup @0x422C + @0x4334):
 *
 *   - console on UART0
 *   - secure-boot self check (SID+0x364 status word)
 *   - scan the boot descriptor table for {0x4A000000, size} and verify
 *     the U-Boot DTB appended to the image
 *   - GIC-600 init (dist + rdist + cpuif)
 *   - arisc (E902) firmware load and startup handshake
 *   - chip revision errata: 0x1689: CCU+0x54=0x3180; both: CCU+0x5C=1
 */

#include <assert.h>

#include <platform_def.h>

#include <arch.h>
#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/desc_image_load.h>
#include <libfdt.h>
#include <drivers/arm/gicv3.h>
#include <drivers/console.h>
#include <drivers/generic_delay_timer.h>
#include <drivers/ti/uart/uart_16550.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <sunxi_def.h>
#include <sunxi_mmap.h>
#include <sunxi_private.h>
#include <sunxi_mini_fdt.inc>


static entry_point_info_t bl32_image_ep_info;
static entry_point_info_t bl33_image_ep_info;

static console_t sunxi_console;

static gicv3_driver_data_t sunxi_gic_data;

/* per-core GICR frames; the vendor BSP keeps the same array at 0x480207D0 */
static uintptr_t sunxi_rdistif_base_addrs[PLATFORM_CORE_COUNT];

/*
 * Secure-boot self check, faithful to  (return codes
 * 0..4).  The image key @0x1286D is embedded below; BROM+0x24 supplies
 * the 8-byte chip info; unlock/verify use SID+0x364 and SID+0x200.
 *
 * The reference caller (0x4358) prints and jumps into its error helpers
 * (0x3F84/0x4D60/0x3090 = console init + panic paths) on a non-zero
 * result.  We log instead of halting so a false positive cannot brick
 * a board; documented difference.
 */
static const uint8_t sunxi_secure_key[16] = {
	0x41, 0x57, 0x55, 0xaa, 0x01, 0x02, 0x03, 0x04,
	0x04, 0x03, 0x02, 0x01, 0xa1, 0xb1, 0xa3, 0xb3,
};

static int sunxi_secure_boot_check(void)
{
	uint8_t info[8];
	uint8_t dec[16] = { 0 };
	uint32_t socid, state, ver;
	uint16_t magic;
	unsigned int i, j;

	for (i = 0U; i < 8U; i++)
		info[i] = mmio_read_8(0x24U + i);	/* BROM chip info */

	state = mmio_read_32(SUNXI_SID_BASE + 0x364U);
	if ((state >> 24) != 0U) {
		ERROR("hardware error 4\n");
		return 4;
	}

	socid = mmio_read_32(SUNXI_SID_BASE + 0x200U);
	if ((socid & 0xffffU) == 0U)
		return 0;				/* not burned */

	ver = info[1] & 0x0fU;
	magic = (uint16_t)info[2] | ((uint16_t)info[3] << 8);
	if (magic != 0x5741U || ver > 4U) {
		ERROR("hardware check error1\n");
		return 1;
	}
	if ((info[1] & 0xf0U) != 0x50U) {
		ERROR("hardware check error2\n");
		return 2;
	}

	/* decrypt `ver` bytes of BROM+0x28 with the 16-byte key */
	for (i = 0U; i < ver; i++) {
		uint8_t v = info[4U + i];

		for (j = 0U; j < 16U; j++)
			v ^= sunxi_secure_key[j];
		dec[i] = v;
	}

	if ((socid & 0xff00U) != 0U) {
		for (i = 0U; i + 1U < ver; i += 2U) {
			if (dec[i] == ((socid >> 8) & 0xffU) &&
			    dec[i + 1U] == (socid & 0xffU))
				return 0;
		}
	} else {
		for (i = 0U; i < ver; i++) {
			if (dec[i] == (socid & 0xffU))
				return 0;
		}
	}

	ERROR("data: %x %x %x %x\n", dec[0], dec[1], dec[2], dec[3]);
	ERROR("hardware error 3\n");
	return 3;
}

/*
 * U-Boot DTB discovery, matching : walk the boot
 * descriptor table left by boot0, locate the {0x4A000000, size} entry
 * and expect the DTB right after the U-Boot image.
 */
void *sunxi_find_uboot_dtb(void)
{
	const uint32_t *cursor = (const uint32_t *)(uintptr_t)SUNXI_BOOT_TABLE_BASE;
	const uint32_t *end = (const uint32_t *)(uintptr_t)(SUNXI_BOOT_TABLE_BASE +
							     SUNXI_BOOT_TABLE_SIZE);

	/* -0x4408: scan in 8-byte steps for the U-Boot load
	 * address, take the word 8 bytes later as size (limit 0x0FFFFFFF)
	 * and check boot_table_base + size for an FDT. */
	for (; cursor < end; cursor += 2) {
		uint32_t size;
		void *dtb;

		if (cursor[0] != (uint32_t)SUNXI_BL33_BASE)
			continue;
		size = cursor[2];
		if (size > 0x0FFFFFFFU)
			continue;

		dtb = (void *)(uintptr_t)(SUNXI_BOOT_TABLE_BASE + size);
		if (fdt_check_header(dtb) == 0) {
			const char *model = (const char *)fdt_getprop(
				dtb, 0, "model", NULL);

			NOTICE("BL31: Found U-Boot DTB at %p, model: %s\n",
			       dtb, model ? model : "unknown");
			return dtb;
		}
	}

	NOTICE("BL31: No DTB found.\n");
	return NULL;
}


void bl31_early_platform_setup2(u_register_t arg0, u_register_t arg1,
				u_register_t arg2, u_register_t arg3)
{
	/* sunxi_ccu_restore(); */

	/*
	 * The 16550 driver derives its divisor from SUNXI_UART0_CLK_IN_HZ
	 * (24 MHz), but boot0 may leave APB_UART_CLK on PERI0_600M_BUS/M
	 * (100 MHz on this board).  Every byte BL31 sends before something
	 * re-inits the UART then arrives ~4x too fast, so the host decodes
	 * a few characters of noise and drops the rest as framing errors -
	 * the first OP-TEE hand-off messages disappeared this way.  Pin the
	 * debug UART to the 24 MHz oscillator: APB_UART_CLK = SYS_CLK24M/1.
	 */
	mmio_write_32(SUNXI_CCU_BASE + 0x538U, 0U);	/* APB_UART_CLK_REG: SYS_CLK24M/1 */

	/*
	 * boot0 can hand over with UART0 still mid-transfer and its divisor
	 * in an unknown state; TF-A's 16550 init neither waits for the
	 * transmitter to drain nor clears the FIFOs, so LSR can read 0 for
	 * the whole early hand-off and every character times out.  Drain
	 * (bounded), reprogram 8n1/115200 from the 24 MHz source and reset
	 * both FIFOs before TF-A's driver owns the port.
	 */
	{
		uint32_t i;

		for (i = 0U; i < 100000U; i++) {
			if ((mmio_read_32(SUNXI_UART0_BASE + 0x14U) & 0x40U) != 0U)
				break;
		}
		mmio_write_32(SUNXI_UART0_BASE + 0x0CU, 0x83U);	/* 8n1, DLAB */
		mmio_write_32(SUNXI_UART0_BASE + 0x00U, 13U);	/* DLL */
		mmio_write_32(SUNXI_UART0_BASE + 0x04U, 0U);	/* DLM */
		mmio_write_32(SUNXI_UART0_BASE + 0x0CU, 0x03U);	/* 8n1 */
		mmio_write_32(SUNXI_UART0_BASE + 0x08U, 0x07U);	/* FIFO, TX/RX clear */
	}

	console_16550_register(SUNXI_UART0_BASE, SUNXI_UART0_CLK_IN_HZ,
			       SUNXI_UART0_BAUDRATE, &sunxi_console);
	/*
	 * The driver enables DMA mode in FCR; this UART has no DMA master
	 * attached and the FIFO then garbles the first characters after
	 * boot0 hands over, so keep FIFO mode with both queues cleared.
	 */
	mmio_write_32(SUNXI_UART0_BASE + 0x08U, 0x07U);
	console_set_scope(&sunxi_console, CONSOLE_FLAG_BOOT |
			  CONSOLE_FLAG_RUNTIME | CONSOLE_FLAG_CRASH);


	/* Apply the CPU errata the vendor monitor applies on every core
	 * (no BL1 in this chain would do it otherwise). */
	sunxi_apply_cpu_errata();

	/*
	 * boot0 fills the monitor header before jumping here
	 * (boot0):
	 *   +0x0C scp_base      (E902 firmware DRAM copy)
	 *   +0x10 uboot_base    -> BL33 entry point
	 *   +0x18 optee_base    -> BL32 entry point (when packaged)
	 * Fall back to the well-known constant if the header was not
	 * populated (bare-metal entry for debugging).
	 */
	{
		uintptr_t hdr = SUNXI_BL31_BASE;	/* container header page */
		uint32_t uboot = mmio_read_32(hdr + 0x10U);
		uint32_t optee = mmio_read_32(hdr + 0x18U);

		/*
		 * BL32 (OP-TEE), -0x41E8: three-way entry
		 * code-type detection; the entry_point_info header carries
		 * type/version/size exactly like the firmware's
		 * SET_PARAM_HEAD (type 1 | version 1 << 8 | size 0x58 << 16).
		 *
		 * SUNXI_SKIP_OPTEE (default OFF): the boot package carries a
		 * real OP-TEE image (TOC1 "optee" item, optee_sun60iw2p1.bin,
		 * boot0 loads it and records optee_base in the monitor header
		 * +0x18).  Hand it to opteed as BL32.  Define SUNXI_SKIP_OPTEE=1
		 * to skip BL32 (falls through to BL33).  NOTE: platform.mk
		 * CFLAGS overrides never reach the compile (TF-A redefines
		 * CFLAGS := after including platform.mk), so the default lives
		 * here.
		 */
#ifndef SUNXI_SKIP_OPTEE
#define SUNXI_SKIP_OPTEE 0
#endif
		if (!SUNXI_SKIP_OPTEE &&
		    optee >= SUNXI_DRAM_BASE && optee < 0x80000000ULL) {
			uint32_t insn = mmio_read_32(optee);

			SET_PARAM_HEAD(&bl32_image_ep_info, PARAM_EP,
				       VERSION_1, 0);
			bl32_image_ep_info.pc = optee;
			SET_SECURITY_STATE(bl32_image_ep_info.h.attr, SECURE);

			if ((insn & 0xFC000000U) == 0x14000000U) {
				/* AArch64 B: EL1h, context id 0 */
				bl32_image_ep_info.spsr = SPSR_64(MODE_EL1,
					MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
				bl32_image_ep_info.args.arg0 = 0U;
				for (uint32_t fi = 0U;
				     fi < sizeof(sunxi_mini_fdt); fi++)
					mmio_write_8(0x4E000000UL + fi,
						     sunxi_mini_fdt[fi]);
				flush_dcache_range(0x4E000000UL,
						   sizeof(sunxi_mini_fdt));
				bl32_image_ep_info.args.arg3 = 0x4E000000UL;
				NOTICE("BL31: OP-TEE 64bit detected\n");
			} else if ((insn & 0xFF000000U) == 0xEA000000U) {
				/* AArch32 B: SVC mode (0x1D3), ctx id 1 */
				bl32_image_ep_info.spsr = SPSR_MODE32(
					MODE32_svc, SPSR_T_ARM,
					SPSR_E_LITTLE,
					DAIF_ABT_BIT | DAIF_IRQ_BIT |
					DAIF_FIQ_BIT);
				bl32_image_ep_info.args.arg0 = 1U;
				NOTICE("BL31: OP-TEE 32bit detected\n");
			} else {
				bl32_image_ep_info.pc = 0U;
				NOTICE("BL31: Unknown jump instruction in "
				       "OP-TEE header\n");
			}
		}


		mmio_write_32(0x48000F10UL, optee);
		/*
		 * BL33 (U-Boot), -0x4214: the mode check reads
		 * the monitor container's own jump word at 0x48000000; the
		 * header +0x14/+0xB0 pair is the boot0/kernel handoff
		 * (normally zero on the U-Boot path).
		 */
		SET_PARAM_HEAD(&bl33_image_ep_info, PARAM_EP, VERSION_1, 0);
		bl33_image_ep_info.pc = (uboot >= SUNXI_DRAM_BASE &&
					 uboot < 0x80000000ULL)
					? uboot : SUNXI_BL33_BASE;

		if (mmio_read_32(hdr + 0x14U) != 0U) {
			uint64_t v = mmio_read_64(hdr + 0xB0U);

			if (v != 0U)
				bl33_image_ep_info.args.arg0 = v;
		}

		{
		uint32_t mode_insn =
			mmio_read_32((uint32_t)bl33_image_ep_info.pc);
		NOTICE("BL31: bl33@0x%x[0..7]=%08x %08x %08x %08x (+0x4000=%08x)\n",
		       (uint32_t)bl33_image_ep_info.pc,
		       mode_insn,
		       mmio_read_32((uint32_t)bl33_image_ep_info.pc + 4),
		       mmio_read_32((uint32_t)bl33_image_ep_info.pc + 8),
		       mmio_read_32((uint32_t)bl33_image_ep_info.pc + 0xc),
		       mmio_read_32((uint32_t)bl33_image_ep_info.pc + 0x4000));

		if ((mode_insn & 0xFC000000U) == 0x14000000U) {
			NOTICE("BL31: U-BOOT 64bit detected\n");
			/*
			 * Deliberate deviation from the vendor BSP (which hands
			 * BL33 at EL1h via SPSR 0x3C5 at 0x417C): boot
			 * U-Boot at EL2 so Linux can run at EL2 (KVM).
			 * User-requested; the single intentional
			 * non-aligned item.
			 */
			bl33_image_ep_info.spsr = SPSR_64(MODE_EL2,
				MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
		} else if ((mode_insn & 0xFF000000U) == 0xEA000000U) {
			NOTICE("BL31: U-BOOT 32bit detected\n");
			bl33_image_ep_info.spsr = SPSR_MODE32(MODE32_svc,
				SPSR_T_ARM, SPSR_E_LITTLE,
				DAIF_ABT_BIT | DAIF_IRQ_BIT | DAIF_FIQ_BIT);
		} else {
			NOTICE("BL31: Unknown jump instruction in U-BOOT header\n");
			bl33_image_ep_info.spsr = SPSR_64(MODE_EL2,
				MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
		}
		}
		SET_SECURITY_STATE(bl33_image_ep_info.h.attr, NON_SECURE);
	}

#ifdef SUNXI_BL32_BASE
	SET_PARAM_HEAD(&bl32_image_ep_info, PARAM_EP, VERSION_1, 0);
	bl32_image_ep_info.pc = SUNXI_BL32_BASE;
	bl32_image_ep_info.spsr = SPSR_64(MODE_EL1, MODE_SP_ELX,
					  DISABLE_ALL_EXCEPTIONS);
	SET_SECURITY_STATE(bl32_image_ep_info.h.attr, SECURE);
#endif
}

void bl31_plat_arch_setup(void)
{
	sunxi_configure_mmu_el3(0);
}

void bl31_platform_setup(void)
{
	unsigned int core;

	{
		int sb = sunxi_secure_boot_check();

		if (sb != 0)
			ERROR("BL31: secure boot check result %d\n", sb);
	}

	/* GIC-600, exactly the bases read from the firmware driver data */
	sunxi_gic_data.gicd_base = SUNXI_GICD_BASE;
	sunxi_gic_data.gicr_base = SUNXI_GICR_BASE;
	sunxi_gic_data.rdistif_num = PLATFORM_CORE_COUNT;
	sunxi_gic_data.rdistif_base_addrs = sunxi_rdistif_base_addrs;
	for (core = 0; core < PLATFORM_CORE_COUNT; core++) {
		/* GIC-600 redistributors are 0x20000 apart (RD base 64 KiB +
		 * SGI frame 64 KiB per core; DT: one contiguous 0xFF004-byte
		 * region from 0x03460000, kernel boot log: CPU0 @0x3460000,
		 * CPU1 @0x3480000).  The previous 0x10000 stride made every
		 * warm-path gicv3_rdistif_init(core>=1) target the WRONG
		 * frame - its PPI/SGI defaulting then clobbered the previous
		 * core's SGI frame (tick PPI 27 + all IPIs) while the real
		 * redistributor was never woken.  That is the boot-time SMP
		 * hang: CPU0's timer/IPI delivery dies while it waits for the
		 * first secondary (no second CPU_ON is ever issued and its 5s
		 * completion timeout never fires). */
		sunxi_gic_data.rdistif_base_addrs[core] =
			SUNXI_GICR_BASE + core * (1U << 17);
	}
	/*
	 * No interrupt properties are passed to the GIC
	 * driver (driver data @0xF010: interrupt_props = NULL, num = 0).
	 */
	sunxi_gic_data.interrupt_props = NULL;
	sunxi_gic_data.interrupt_props_num = 0U;

	gicv3_driver_init(&sunxi_gic_data);
	gicv3_distif_init();
	gicv3_rdistif_init(plat_my_core_pos());
	gicv3_cpuif_enable(plat_my_core_pos());

	/* boot-time system restore  */
	sunxi_system_restore();

	/* secure-DRAM / DRM region table  */
	sunxi_secure_mem_init();

	generic_delay_timer_init();

	/* chip revision errata, from  */
	switch (sunxi_read_soc_id()) {
	case SUNXI_SOC_SUN60IW2P1_A:
		mmio_write_32(SUNXI_CCU_BASE + 0x54, 0x3180U);
		mmio_write_32(SUNXI_CCU_BASE + 0x5C, 1U);
		break;
	case SUNXI_SOC_SUN60IW2P1_B:
		mmio_write_32(SUNXI_CCU_BASE + 0x5C, 1U);
		break;
	default:
		WARN("BL31: unknown SoC id 0x%x\n", sunxi_read_soc_id());
		break;
	}

	/*
	 * Auto-start ARISC (E902).  The stock Radxa EDK2 never runs
	 * AriscStartupDxe (stock gap), so without this the ARISC never
	 * starts and PSCI SYSTEM_OFF/RESET dead-loop waiting for an
	 * SYS_OP reply that never comes.
	 *
	 * boot0 maps the whole boot package at 0x42E00000 but never
	 * dispatches the "scp" item (stock packages carry none, so that
	 * path was never exercised).  Self-stage: our package places the
	 * scp item at package offset 0x200000 -> 0x43000000; verify its
	 * E902 magic (0x81400141) and copy to 0x48100000.
	 */
	{
		/* Stage the ARISC "scp" item from the live boot-package map
		 * (boot0 maps the whole package at 0x42E00000 but never
		 * dispatches scp), then auto-start the E902.  TOC1 layout:
		 * +0x00 "sunxi-package", +0x20 items_nr, entries 0x170B
		 * starting at +0x40 with name[64] / +0x40 data_offset /
		 * +0x44 data_len.  Stock Radxa EDK2 has no AriscStartupDxe
		 * (stock gap), so without this the ARISC never starts and
		 * PSCI SYSTEM_OFF/RESET dead-loop on the unanswered SYS_OP. */
		if (mmio_read_32(SUNXI_ARISC_LOAD_BASE) == 0U &&
		    mmio_read_32(0x42E00010UL) == 0x89119800U) {
			const uint8_t *pkg = (const uint8_t *)0x42E00000UL;
			uint32_t nr = mmio_read_32(0x42E00020UL);
			uint32_t i;

			for (i = 0U; i < nr && i < 16U; i++) {
				const uint8_t *e = pkg + 0x40U + i * 0x170U;
				uint32_t off, len;

				if (memcmp(e, "scp", 3) != 0)
					continue;
				off = mmio_read_32((uintptr_t)e + 0x40U);
				len = mmio_read_32((uintptr_t)e + 0x44U);
				if (len == 0U || len > 160U * 1024U)
					break;
				NOTICE("BL31: arisc scp staged (TOC1)\n");
				memcpy((void *)SUNXI_ARISC_LOAD_BASE,
				       pkg + off, len);
				flush_dcache_range(SUNXI_ARISC_LOAD_BASE, len);
				break;
			}
		}
		/* ARISC startup is NOT done here.  The vendor BL31's
		 * sunxi_arisc_init (0xAF64) has a single caller: SIP FID
		 * 0x8000FF10, invoked by EDK2's AriscStartupDxe with the
		 * correct dtb address.  Earlier ports auto-started the E902 at
		 * BL31 boot with boot_arg=0 (dtb=0), which made the E902 parse
		 * address 0 and stall, and a later FF10 collided with the
		 * already-running core.  We only stage the firmware here; the
		 * DXE-driven FF10 does the copy/release/handshake. */
	}
}

void bl31_plat_runtime_setup(void)
{
	/* Keep the boot console alive into the runtime scope, matching the
	 * vendor BSP. */
	console_switch_state(CONSOLE_FLAG_BOOT);

	NOTICE("BL31: runtime setup done\n");
}

entry_point_info_t *bl31_plat_get_next_image_ep_info(unsigned int type)
{
	entry_point_info_t *next = (type == NON_SECURE) ? &bl33_image_ep_info
							: &bl32_image_ep_info;

	if ((type == SECURE) && (next->pc == 0U) && !SUNXI_SKIP_OPTEE) {
		/* boot0 descriptor "secureos" entry.  When the boot
		 * package carries an OP-TEE image (TOC1 item "optee",
		 * e.g. device-a733/bin/optee_sun60iw2p1.bin, linked at
		 * 0x4C000000), the reference boot0 loads it to this base and
		 * records {0x4C000000, size} in the boot descriptor table
		 * (same table walked by sunxi_find_uboot_dtb).  Gate on the
		 * image magic ("optee" @+4) so a package without a TEE
		 * keeps the no-BL32 fallback path (opteed continues to
		 * BL33 - already deviated upstream behaviour). */
		const uint32_t *cur = (const uint32_t *)(uintptr_t)SUNXI_BOOT_TABLE_BASE;
		const uint32_t *end = (const uint32_t *)(uintptr_t)(SUNXI_BOOT_TABLE_BASE +
								     SUNXI_BOOT_TABLE_SIZE);

		for (; cur < end; cur += 2) {
			if (cur[0] != 0x4C000000U)
				continue;
			{
				const uint8_t *img = (const uint8_t *)(uintptr_t)0x4C000000UL;
				uint32_t size = cur[1];

				if (size == 0U || size > 0x0FFFFFFFU)
					break;
				if (img[0] == 0x00U && img[1] == 0x04U &&
				    img[2] == 0x00U && img[3] == 0x14U &&
				    img[4] == 'o' && img[5] == 'p' &&
				    img[6] == 't' && img[7] == 'e' &&
				    img[8] == 'e') {
					SET_PARAM_HEAD(next, PARAM_EP, VERSION_1,
						       EP_SECURE);
					next->pc = 0x4C000000UL;
					next->spsr = SPSR_64(MODE_EL1,
							     MODE_SP_ELX,
							     DISABLE_ALL_EXCEPTIONS);
					next->args.arg0 = 0U;
					next->args.arg1 =
						(uintptr_t)sunxi_find_uboot_dtb();
					next->args.arg2 = 0U;
					next->args.arg3 = 0U;
					NOTICE("BL31: OP-TEE found at 0x4C000000"
					       " (%u B)\n", size);
					return next;
				}
				break;
			}
		}
		return NULL;
	}
	return next;
}

/*
 * Warm-entry MMU hook (bl31_warm_entrypoint -> el3_enable_mmu): the
 * translation tables already exist by then, so only the MMU itself may be
 * enabled.  Rebuilding them here re-enters mmap_add_region() with an
 * initialised xlat context and asserts - which is exactly what killed the
 * secondary core on PSCI CPU_ON (entry = bl31_warm_entrypoint).
 */
void bl31_plat_enable_mmu(uint32_t flags)
{
	enable_mmu_el3(flags);
}
