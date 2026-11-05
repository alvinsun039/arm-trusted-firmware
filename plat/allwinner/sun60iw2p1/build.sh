#!/bin/sh
# Copyright (c) 2026, A733 open-source BL31 contributors
# SPDX-License-Identifier: BSD-3-Clause
#
# Build the open-source A733 BL31.  The raw image is the whole product:
# wrapping it into the "monitor" container and into a boot_package is the
# boot-package packer's job, so this script stays a plain TF-A
# build and needs no vendor artifact.
#
# usage: ./build.sh [make VAR=val ...]
#   ./build.sh                       release build
#   ./build.sh LOG_LEVEL=50 DEBUG=1  full debug build (build/sun60iw2p1/debug)
#
# output: build/sun60iw2p1/<release|debug>/bl31.bin
set -e

TF_A_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
PLAT_DIR="${TF_A_ROOT}/plat/allwinner/sun60iw2p1"
CROSS_COMPILE="${CROSS_COMPILE:-aarch64-linux-gnu-}"

# TF-A picks the output directory from DEBUG (release unless DEBUG != 0).
MODE=release
for a in "$@"; do
	case "$a" in
	DEBUG=0|DEBUG=)	MODE=release ;;
	DEBUG=*)	MODE=debug ;;
	esac
done
OUT="${TF_A_ROOT}/build/sun60iw2p1/${MODE}"

echo "== 1. build TF-A bl31 (PLAT=sun60iw2p1, SPD=opteed, ${MODE}) =="
make -C "${TF_A_ROOT}" PLAT=sun60iw2p1 SPD=opteed \
	CROSS_COMPILE="${CROSS_COMPILE}" -j"$(nproc)" "$@" bl31

echo
echo "Artifact:"
echo "  ${OUT}/bl31.bin"
echo
echo "Wrap it into a boot_package with the boot-package packer."
echo "(--scp/--scp-dts/--optee optional)."