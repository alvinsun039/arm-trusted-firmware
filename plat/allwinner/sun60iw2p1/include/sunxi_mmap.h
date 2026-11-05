/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * sun60iw2p1 (Allwinner A733) memory map.
 * Verified against the A733 User Manual (V0.92) ch.2 and the reference
 * the vendor BSP (TF-A v2.5(debug):48e54578a).
 */

#ifndef SUNXI_SUN60IW2P1_MMAP_H
#define SUNXI_SUN60IW2P1_MMAP_H

/* DRAM */
#define SUNXI_DRAM_BASE			0x40000000ULL
#define SUNXI_DRAM_VIRT_BASE		0x40000000ULL

/* Boot stages */
#define SUNXI_SRAM_BASE			0x00000000
#define SUNXI_SRAM_A2_BASE		0x00040000
#define SUNXI_SRAM_A2_SIZE		0x00034000

/* BL31 is loaded by boot0 at 0x48000000 (monitor container header) */
#define SUNXI_BL31_BASE			0x48000000
#define SUNXI_BL31_SIZE			0x000C0000	/* 768 KiB (USB port BSS) */
/* per-CPU stacks 0x48014400 + core*0x1000 are inside the image area */

/* Next boot stages */
#define SUNXI_BL33_BASE			0x4A000000	/* U-Boot */
/* boot descriptor table left by boot0, 2KiB of {addr,size} pairs */
#define SUNXI_BOOT_TABLE_BASE		0x04000000
#define SUNXI_BOOT_TABLE_SIZE		0x00000800

/* E902 (arisc/CPUS) firmware windows */
#define SUNXI_ARISC_SRAM_BASE		0x00040000	/* SRAM_A2 base (BL31 view) */
#define SUNXI_ARISC_LOAD_BASE		0x48100000	/* boot0 load addr */
#define SUNXI_ARISC_PARA_LOAD_BASE	0x4812C000
#define SUNXI_ARISC_ENTRY		0x40004000	/* E902 view of 0x44000  */
#define SUNXI_ARISC_SRAM_SIZE		0x00028000	/* firmware copy size  */

/* Peripherals - system */
#define SUNXI_SYSCTRL_BASE		0x03000000
#define SUNXI_MSGBOX_CPUX_BASE		0x03004000	/* CPUS->CPUX box */
#define SUNXI_SPINLOCK_BASE		0x03005000
#define SUNXI_SID_BASE			0x03006000
#define SUNXI_GICD_BASE			0x03400000
#define SUNXI_GICR_BASE			0x03460000

/* Peripherals - display/media (not used by BL31, documented) */
#define SUNXI_DE0_BASE			0x05000000
#define SUNXI_HDMI_BASE			0x05520000

/* Peripherals - CPU system (secure) */
#define SUNXI_S_PRCM_BASE		0x07010000
#define SUNXI_CPUS_E902_CFG_BASE	0x07032000
#define SUNXI_CPUIDLE_BASE		0x07050000
#define SUNXI_PPU_BASE			0x07060000
#define SUNXI_MSGBOX_CPUS_BASE		0x07094000	/* CPUX->CPUS box */
#define SUNXI_RTC_BASE			0x07090000
#define SUNXI_RINTC_BASE		0x07024000	/* R_INTC (always-on wake ctrl) */

/* Peripherals - CPU subsystem */
#define SUNXI_GPIO_BASE			0x02000000
#define SUNXI_SRAM_CTRL_BASE		0x03000000	/* SoC ver / SRAM ctrl	*/
#define SUNXI_NSI_BASE			0x02020000	/* NSI (64K)		*/
/* words saved/restored across suspend  */
#define SUNXI_NSI_SUSPEND_W0		0x02000100
#define SUNXI_NSI_SUSPEND_W1		0x02000300
#define SUNXI_CCU_BASE			0x02002000
#define SUNXI_USB0_BASE			0x04100000	/* USB2.0 DRD (device) */
#define SUNXI_SMC0_BASE			0x0A000000	/* DRAM controller 0 (64K) */
#define SUNXI_CPU_SUBSYS_BASE		0x08000000	/* RVBAR/AA64nAA32 */
#define SUNXI_CLUSTER_CFG_BASE		0x08860000	/* PCK600 cluster */

/* Timers */
#define SUNXI_CNTBASE_BASE		0x02052000	/* TIMER1 */

/* Generic region bookkeeping required by the common platform layout */
#define SUNXI_SRAM_SIZE			0x00080000
#define SUNXI_DEV_BASE			0x01000000
#define SUNXI_DEV_SIZE			0x0F000000
#define SUNXI_DRAM_MAP_SIZE		(64U << 20)
#define SUNXI_BL33_VIRT_BASE		SUNXI_BL33_BASE
#define SUNXI_SYSCON_BASE		SUNXI_SYSCTRL_BASE

#endif /* SUNXI_SUN60IW2P1_MMAP_H */
