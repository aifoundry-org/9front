#!/bin/bash
#
# Point an already-flashed Banana Pi F3 eMMC at a different 9front kernel.
# Companion to flash-emmc-uboot-and-partitions.sh: that script lays out the
# whole eMMC; this one only rewrites the kernel filename inside the u-boot
# bootcmd (env partition p3) and, if KERNEL is given, copies the file onto
# the existing 9fat partition (p4).  GPT, firmware and the hw boot
# partitions are not touched, so there is nothing to lose but the env.
#
# The rest of bootcmd (plan9.ini load, DTB, fallback bootargs, booti) is
# preserved as-is by editing the current value in place rather than
# regenerating it.
#
# Usage:
#   sudo bash switch-emmc-kernel.sh                     # bootcmd -> 9rv64
#   sudo NAME=9bpif3 bash switch-emmc-kernel.sh         # switch back
#   sudo KERNEL=/path/9rv64 bash switch-emmc-kernel.sh  # copy + switch
#
# NAME defaults to the basename of KERNEL when that is set, else 9rv64.
#

set -eu

DEV=${DEV:-/dev/mmcblk2}
KERNEL=${KERNEL:-}
NAME=${NAME:-${KERNEL:+$(basename "$KERNEL")}}
NAME=${NAME:-9rv64}

die() { echo "switch-emmc-kernel: $*" >&2; exit 1; }

[ "$(id -u)" = 0 ] || die "must run as root"
command -v fw_setenv >/dev/null || die "fw_setenv not found (apt install libubootenv-tool)"

[ -b "$DEV" ] || die "$DEV is not a block device"
# same eMMC sanity checks as the flash script: hw boot partitions exist,
# and the name doesn't look like the SD card.
[ -b "${DEV}boot0" ] || die "$DEV has no boot0 hw partition; not the eMMC"
case "$DEV" in
*mmcblk0*) die "$DEV looks like the SD card; refusing" ;;
esac
[ -b "${DEV}p3" ] || die "${DEV}p3 (env) missing; run the flash script first"
[ -b "${DEV}p4" ] || die "${DEV}p4 (9fat) missing; run the flash script first"

if [ -n "$KERNEL" ]; then
	[ -r "$KERNEL" ] || die "KERNEL='$KERNEL' unreadable"
	command -v mcopy >/dev/null || die "mcopy not found (apt install mtools)"
fi

tmpcfg=$(mktemp)
trap 'rm -f "$tmpcfg"' EXIT
printf '%s 0x0 0x4000\n' "${DEV}p3" >"$tmpcfg"

bootcmd=$(fw_printenv -c "$tmpcfg" -n bootcmd) || die "no bootcmd in env; run the flash script first"
case "$bootcmd" in
*"load mmc 2:4 0x50000000 "*) ;;
*) die "bootcmd has no 'load mmc 2:4 0x50000000 <kernel>' stage; not editing it blind" ;;
esac

newcmd=$(sed 's/load mmc 2:4 0x50000000 [^;]*;/load mmc 2:4 0x50000000 '"$NAME"';/' <<<"$bootcmd")

echo "=== $DEV ==="
echo "current bootcmd:"
echo "    $bootcmd"
echo "new bootcmd:"
echo "    $newcmd"
[ -n "$KERNEL" ] && echo "will copy $KERNEL to 9fat as $NAME"
if [ "$newcmd" = "$bootcmd" ] && [ -z "$KERNEL" ]; then
	echo "nothing to do."
	exit 0
fi
read -r -p "type YES to proceed: " confirm
[ "$confirm" = YES ] || die "aborted"

if [ -n "$KERNEL" ]; then
	echo
	echo "=== copy kernel onto 9fat ==="
	mcopy -o -i "${DEV}p4" "$KERNEL" ::"$NAME"
fi

# make sure u-boot will actually find the file before repointing bootcmd
if command -v mdir >/dev/null; then
	mdir -i "${DEV}p4" -b | grep -qix "::/$NAME" ||
		die "$NAME not on the 9fat partition; copy it first (KERNEL=/path/$NAME)"
fi

echo
echo "=== update env ==="
fw_setenv -c "$tmpcfg" bootcmd "$newcmd"
echo "env now says:"
fw_printenv -c "$tmpcfg" bootcmd | sed 's/^/    /'

echo
echo "done.  next boot loads $NAME from the 9fat partition."
