/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SUNXI_DEF_H
#define SUNXI_DEF_H

#define SUNXI_SOC_SUN60IW2P1_A		0x1689U
#define SUNXI_SOC_SUN60IW2P1_B		0x1718U

/* JEDEC identifiers for SMCCC_ARCH_SOC_ID */
#define JEDEC_ALLWINNER_BKID		U(9)
#define JEDEC_ALLWINNER_MFID		U(0x9e)

/* Chip ID lives in the SID user region */
#define SUNXI_SID_SOC_ID_REG		(SUNXI_SID_BASE + 0x200)


/* E902 arisc/CPUS parameters */
/* see sunxi_arisc.c - 4s bounded waits aborted mid-frame on a
 * full queue and left partial frames that desynced the ARISC reader
 * permanently.  Mirror the vendor BSP's effectively-unbounded
 * wait with a huge finite bound. */
#define SUNXI_ARISC_TIMEOUT_MS		120000U

#endif /* SUNXI_DEF_H */
