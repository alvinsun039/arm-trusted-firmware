#!/usr/bin/env bash
# Copyright (c) 2026, A733 open-source BL31 contributors
# SPDX-License-Identifier: BSD-3-Clause
#
# build.sh - build the sun60iw2p1 (A733) BL31 with the OP-TEE SPD.
#
#   ./build.sh                       release build
#   ./build.sh DEBUG=1               debug build
#   CROSS_COMPILE=aarch64-none-elf- ./build.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLAT=sun60iw2p1
SPD="${SPD:-opteed}"
JOBS="$(nproc)"
CROSS_COMPILE="${CROSS_COMPILE:-aarch64-linux-gnu-}"

MODE=release
for a in "$@"; do
	case "$a" in
	DEBUG=0|DEBUG=) MODE=release ;;
	DEBUG=*) MODE=debug ;;
	esac
done

echo "== tf-a: building BL31 (PLAT=$PLAT SPD=$SPD, $JOBS jobs, $MODE)"
make -C "$SCRIPT_DIR" PLAT="$PLAT" SPD="$SPD" CROSS_COMPILE="$CROSS_COMPILE" \
  -j"$JOBS" "$@" bl31

BL31="$SCRIPT_DIR/build/$PLAT/$MODE/bl31.bin"
if [ ! -f "$BL31" ]; then
  echo "ERROR: BL31 not produced: $BL31" >&2
  exit 1
fi
echo "== tf-a: done -> $BL31 ($(stat -c%s "$BL31") B)"
