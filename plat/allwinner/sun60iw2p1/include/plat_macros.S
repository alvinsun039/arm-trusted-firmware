/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLAT_MACROS_S
#define PLAT_MACROS_S

#include <platform_def.h>

	/* ---------------------------------------------
	 * The A733 uses a GIC-600 (GICv3). The generic
	 * crash printer only knows GICv2, so keep the
	 * platform crash report minimal and print the
	 * distributor type register only.
	 * Clobbers: x0 - x10, x16, x17, sp
	 * ---------------------------------------------
	 */
	.macro plat_crash_print_regs
	/* nothing printed - gicv3 state is saved by the EHF dump */
	.endm

#endif /* PLAT_MACROS_S */
