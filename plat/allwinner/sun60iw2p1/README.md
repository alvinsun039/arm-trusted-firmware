<!-- Copyright (c) 2026, A733 open-source BL31 contributors -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# sun60iw2p1 — BL31 for the Allwinner A733 (Radxa Cubie A7A)

An out-of-tree arm-trusted-firmware platform port for the Allwinner A733
(sun60iw2p1): BL31 runs as the monitor stage loaded by boot0 and hands
control of the next stage.  Register and flow references are the A733 User
Manual (V0.92) and the E902 firmware's message definitions; no vendor binary
image is an input to this build.

## What is implemented

| Feature | Status | Notes |
|---|---|---|
| Cold boot from boot0 (monitor container) | yes | image linked at 0x48001000, header page at 0x48000000 |
| UART console (UART0 @0x02500000, 24MHz, 115200) | yes | same port as the bootloader |
| GIC-600 driver (GICD 0x03400000, GICR 0x03460000) | yes | SPI/PPI configuration for the NS world |
| PSCI: CPU_ON / CPU_OFF / STANDBY / SYSTEM_OFF / SYSTEM_RESET | yes | via CPUIDLE PPU + CPU_SUBSYS RVBAR |
| CPU idle (CPU_SUSPEND at core/cluster level) | yes | WFI in place; the powerdown request is downgraded to retention in `plat_psci_ops.validate_power_state` |
| arisc (E902) bring-up + hwmsgbox channel 3 | yes | message protocol per manual 6.1 (MSGBOX) |
| sunxi SIP SMCs (OEN 0, vendor FIDs) | yes | arisc-based commands; efuse read-only |
| Boot descriptor / DTB discovery + "model" print | yes | boot descriptor table @0x04000000 |
| chip revision errata (CCU 0x54/0x5C) | yes | chipid 0x1689 / 0x1718 |
| SMCCC 1.2 / SOC_ID through the same OEN-0 entry | yes | forwarded through the vendor SIP handler |
| efuse service (kernel ABI 0xC000FE00/FE01/FF05/FF06/FF07/FE03) | yes | SID direct read; write refused by design (irreversible) |
| COPY_ARISC_PARAS (0xC000FF10-class) | yes | validated NS-to-NS DRAM copy |
| system suspend | partial | E902 ARISC super-standby: the AP cluster and DRAM power down and the core resumes through the PSCI warm path, but it is not reliable yet. The suspend side (CPU_OP hand-off, power-down, BL31 resume) is in place and a full cycle has been observed; the remaining failures are the E902 not waking from the standby when the external RTC alarm fires, and the kernel occasionally stalling in its own resume path |
| REBOOT_EFEX (0xC000FF16) | yes | RTC fel flag 0x5A @0x07090118 + reboot; boot0 takes over |
| GET_DRM_INFO / GET_TEEADDR / INFORM_FDT | n/a | these live in OP-TEE (OEN 50), not BL31; kernel degrades gracefully when TEE absent |

## Layout

    platform.mk            build glue (GIC-600, Cortex-A76/A55, RESET_TO_BL31)
    include/platform_def.h BL31_BASE = 0x48001000 (one 4K page after header)
    include/sunxi_mmap.h   memory map (manual ch. 2)
    include/sunxi_def.h    SoC ids, SID regs, JEDEC ids
    include/sunxi_private.h
    include/plat_macros.S
    plat_helpers.S         plat_my_core_pos = MPIDR.Aff1 (flat 8 cores)
    sunxi_bl31_setup.c     boot flow (console, DTB, GIC, arisc, errata)
    sunxi_common.c         MMU tables, SoC id (SID+0x200)
    sunxi_topology.c       1 cluster x 8 cores, index = Aff1
    sunxi_cpu_ops.c        PPU state, RVBAR, AA64NAA32 (manual 5.1)
    sunxi_pm.c             PSCI ops table and the suspend/resume path
    sunxi_arisc.c          E902 loader + hwmsgbox transport
    sunxi_sip_svc.c        vendor SIP dispatch + SMCCC forwarding
    sunxi_efuse.c          SID service for the vendor kernel ABI
    build.sh               one-shot build

## Build

From the top of the tree:

    ./build.sh                     # release build
    ./build.sh DEBUG=1             # debug build
    CROSS_COMPILE=aarch64-none-elf- ./build.sh

The script uses `SPD=opteed` and defaults to the
`aarch64-linux-gnu-` cross compiler; override `CROSS_COMPILE` for
another toolchain.  `plat/allwinner/sun60iw2p1/build.sh` runs the same
build from inside the platform directory.

The artifact is:

    build/sun60iw2p1/release/bl31.bin    raw image (linked 0x48001000)

Wrapping the raw image into the Allwinner "monitor" container and packing it
with the rest of the boot chain is the boot-package packer's job: it takes
this raw `bl31.bin` directly and needs no vendor artifact.

Expected console output:

    BL31: waiting for arisc startup notification
    BL31: arisc started, version 0x...
    BL31: Found U-Boot DTB at 0x..., model: ...
    BL31: sunxi SIP service registered

## Verification hooks

* `bl31_entrypoint` is linked at `0x48001000`; the packer's monitor container
  begins with `b 0x1000` so boot0's jump-to-load-address lands in the header
  page and falls through into the image — no relocation needed.
* A boot_package stores BL31 as a TOC1 entry `{id=3, len, lead=0x3b}`; the
  packer writes the container into that entry, and the entry length is the
  container length, so a larger BL31 means repacking rather than patching.
* The SD-boot path was validated on the board; the package patch step takes
  any existing TOC1 package, so no particular one is required to build this
  port.  Secure-boot-enabled boards (efuse ROTPK burned) will reject the
  unsigned image — the SID self-check (`SID+0x364`, bit pattern 0x50) reports
  status.

## Provenance

* register bitfields: A733_User_Manual_V0.92.pdf (ch. 2, 4.2, 5.1, 5.2, 6.1)
* arisc message protocol: the E902 firmware's message/frame definitions
* efex flashing: boot0's FES stage, reached via the RTC fel flag
  (0x5A @ RTC+0x118) written before reboot
* structure modeled after upstream `plat/rockchip/rk3588` (the RK3588
  open-BL31 precedent) and `plat/allwinner/common` (GICv2 platforms)