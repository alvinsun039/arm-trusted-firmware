/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SUNXI_PRIVATE_H
#define SUNXI_PRIVATE_H

#include <lib/psci/psci.h>
#include <platform_def.h>

/* sunxi_bl31_setup.c */
void sunxi_configure_mmu_el3(int flags);
void *sunxi_find_uboot_dtb(void);

/* sunxi_cpu_ops.c */
void sunxi_cpu_on(u_register_t mpidr);
void sunxi_cpu_off(void);
bool sunxi_cpu_is_powered(unsigned int core);

/* sunxi_pm.c */
void sunxi_system_restore(void);

/* sunxi_arisc.c */
#define ARISC_SYS_SHUTDOWN		U(0)
#define ARISC_SYS_REBOOT		U(1)
#define ARISC_SYS_RESET			U(2)
#define ARISC_SYS_UBOOT_SHUTDOWN	U(3)
#define ARISC_POWER_ON			U(0)
#define ARISC_POWER_RETENTION		U(1)
#define ARISC_POWER_OFF			U(3)

#define SUNXI_BP243_NO_SRE 1

int sunxi_arisc_init(uint32_t boot_arg);
int sunxi_arisc_msg_send(uint32_t type, const uint32_t *paras,
			 unsigned int count);
void sunxi_arisc_rx_drain(void);
int sunxi_arisc_sys_op(uint32_t state);
int sunxi_arisc_standby_restore_notify(void);
int sunxi_arisc_cpu_op(u_register_t mpidr, uintptr_t resume_entry,
		       uint32_t cpu_state, uint32_t cluster_state,
		       uint32_t system_state);

/* sunxi_pm.c */
extern uint32_t sunxi_standby_result;

/* sunxi_common.c */
void sunxi_ccu_restore(void);

/* sunxi_drm.c */
void sunxi_drm_check_region(void);
void sunxi_secure_mem_init(void);
int sunxi_drm_range_is_secure(uint64_t addr, uint64_t size);
int sunxi_drm_ptr_ok(uintptr_t p, uint64_t len);
int sunxi_drm_copy_from_unsafe(uint64_t dst, uint64_t src, uint64_t size);

/* sunxi_common.c / platform glue */
uint16_t sunxi_read_soc_id(void);
uintptr_t sec_entrypoint_get(void);

/* sunxi_cpu_ops.c: per-core CPU errata (cold + warm boot) */
void sunxi_apply_cpu_errata(void);
void sunxi_cluster_power_assist(unsigned int cluster, unsigned int core);
void sunxi_dma_save(void);
void sunxi_dma_restore(void);

/* sunxi_efuse.c - vendor kernel ABI (bsp/drivers/smc/sunxi-smc.c) */
int sunxi_efuse_read(uintptr_t key_buf, uintptr_t read_buf);
int sunxi_efuse_write(uintptr_t key_buf, uintptr_t read_buf);
int sunxi_read_sec_reg(uintptr_t addr, uint32_t *value);
int sunxi_write_sec_reg(uintptr_t addr, uint32_t value);
int sunxi_efuse_read_socid_xored(uintptr_t key_name, uintptr_t out_buf,
				 uintptr_t len);
int sunxi_efuse_probe_secure(void);

/* sunxi_sip_svc.c */
int sunxi_sip_init(void);

#endif /* SUNXI_PRIVATE_H */
