/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A733 power domain topology: one root, one DynamIQ cluster, 8 cores.
 * Core index = MPIDR.Aff1 (see plat_helpers.S).
 */

#include <platform_def.h>

#include <arch.h>
#include <plat/common/platform.h>

static const unsigned char plat_power_domain_tree_desc[PLAT_MAX_PWR_LVL + 1] = {
	/* number of root nodes */
	1,
	/* number of clusters */
	PLATFORM_CLUSTER_COUNT,
	/* number of children of the cluster */
	PLATFORM_MAX_CPUS_PER_CLUSTER,
};

const unsigned char *plat_get_power_domain_tree_desc(void)
{
	return plat_power_domain_tree_desc;
}

int plat_core_pos_by_mpidr(u_register_t mpidr)
{
	unsigned int aff1 = MPIDR_AFFLVL1_VAL(mpidr);
	unsigned int aff2 = MPIDR_AFFLVL2_VAL(mpidr);

	/*
	 * The SoC puts the flat core index in Aff1 (the DT's cpu@0,
	 * cpu@100 .. cpu@700 nodes agree), so Aff0, Aff2 and Aff3 must be
	 * zero; Aff1 is the core number and must be within the cluster.
	 */
	if ((MPIDR_AFFLVL0_VAL(mpidr) != 0U) || (aff2 != 0U) ||
	    (MPIDR_AFFLVL3_VAL(mpidr) != 0U) ||
	    (aff1 >= PLATFORM_MAX_CPUS_PER_CLUSTER)) {
		return -1;
	}

	return (int)aff1;
}

