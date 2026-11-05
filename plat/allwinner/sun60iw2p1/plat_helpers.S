/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <asm_macros.S>
#include <sunxi_mmap.h>

	.globl	plat_my_core_pos
	.globl	platform_mem_init
	.globl	plat_report_exception

/*
 * A733: single DynamIQ cluster of 8 cores; Allwinner maps the logical core
 * number to MPIDR.Aff1 (verified against the vendor BSP plat_my_core_pos @0x37E4
 * and plat_core_pos_by_mpidr @0x82CC).
 */
func plat_my_core_pos
	mrs	x0, mpidr_el1
	ubfx	x0, x0, #MPIDR_AFF1_SHIFT, #MPIDR_AFFINITY_BITS
	ret
endfunc plat_my_core_pos

func platform_mem_init
	ret
endfunc platform_mem_init

func plat_report_exception
	ret
endfunc plat_report_exception