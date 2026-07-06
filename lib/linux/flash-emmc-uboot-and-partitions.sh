#!/bin/bash
#
# Flash SpacemiT K1 u-boot chain to a Banana Pi F3 eMMC and lay out the
# partitions needed for a 9front install.  Runs from a Bianbu Linux userspace
# that already has u-boot-spacemit + opensbi-spacemit packages installed.
#
# End state, SD card removed:
#   BootROM -> FSBL (eMMC boot0/boot1 hw partition)
#          -> OpenSBI (GPT partition "opensbi", offset 1 MiB)
#          -> u-boot  (GPT partition "uboot",   offset 2 MiB)
#          -> u-boot bootcmd loads 9front kernel + DTB from the 9fat partition
#             and jumps into it via 'booti'
#
# Partition table (GPT) written to the eMMC user area:
#   1  opensbi   sectors 2048..4095       1 MiB   fw_dynamic.itb
#   2  uboot     sectors 4096..8191       2 MiB   u-boot.itb
#   3  env       sectors 768..895        64 KiB   env.bin (customized)
#   4  9fat      sectors 16384..212991  ~96 MiB   FAT: kernel + DTB [+ plan9.ini]
#   5  plan9     sectors 212992..end             Plan 9 container (nvram + fs)
#
# The named partitions "opensbi" and "uboot" are what the SpacemiT SPL looks up
# by name; without them the SPL falls through and boot fails.
#
# Env customizations (in addition to the stock env.bin from u-boot-spacemit):
#   bootdelay=3          give a 3s window to break into u-boot
#   mac_mapping_file=''  disable the vendor NFS-lookup that otherwise hangs
#   serverip=''          same reason
#   bootcmd=...          load plan9.ini from the 9fat partition to CONFADDR
#                        and kernel + DTB to staging, set a fallback
#                        /chosen/bootargs, booti into the 9front kernel
#
# Boot configuration lives in plan9.ini on the 9fat partition, one
# key=value per line, same as the PC and the arm u-boot boards: bootcmd
# zeroes CONFADDR (0x40010000) and loads the file there; the kernel
# parses it in place and only falls back to the /chosen/bootargs line
# (baked from $BOOTARGS below) when the file is missing.  The running
# 9front system reconfigures itself by editing a FAT file — no
# fw_setenv, no Linux.
#
# 9fat population: if KERNEL and DTB are set to file paths, the script
# formats p4 as FAT and copies the files there, plus plan9.ini — taken
# from $PLAN9INI if set, otherwise generated from $BOOTARGS.
#
# WARNING: destroys everything on the target device.  Defaults to
# /dev/mmcblk2 (eMMC on the BPI-F3).  Override with $DEV=/dev/whatever.
#
# Usage:
#   sudo bash flash-emmc-uboot-and-partitions.sh
#   sudo KERNEL=/path/9bpif3 DTB=/path/k1-x_deb1.dtb \
#        bash flash-emmc-uboot-and-partitions.sh
#

set -eu

DEV=${DEV:-/dev/mmcblk2}
UBOOT_DIR=${UBOOT_DIR:-/usr/lib/u-boot/spacemit}
OPENSBI_ITB=${OPENSBI_ITB:-/usr/lib/riscv64-linux-gnu/opensbi/generic/fw_dynamic.itb}

FSBL=$UBOOT_DIR/FSBL.bin
BOOTINFO=$UBOOT_DIR/bootinfo_emmc.bin
UBOOT_ITB=$UBOOT_DIR/u-boot.itb
ENV_BIN=$UBOOT_DIR/env.bin

PLAN9_TYPE_GUID=C91818F9-8025-47AF-89D2-F030D7000C2C

# Optional 9front assets.  If set, the script populates the 9fat partition.
KERNEL=${KERNEL:-}
DTB=${DTB:-}
PLAN9INI=${PLAN9INI:-}

# Optional bootargs override.  Default matches the tcp-fs / kexec setup that
# has been used for development; adjust for your environment (fs= etc).
BOOTARGS=${BOOTARGS:-'console=0 nobootprompt=tcp ip=dhcp nvram=/dev/sdM0/nvram nvroff=0 nvrlen=512 fs=10.42.0.9 auth=10.42.0.9 user=glenda service=cpu sysname=bpi'}

die() { echo "flash-emmc: $*" >&2; exit 1; }

[ "$(id -u)" = 0 ] || die "must run as root"

for f in "$FSBL" "$BOOTINFO" "$UBOOT_ITB" "$ENV_BIN" "$OPENSBI_ITB"; do
	[ -r "$f" ] || die "missing input: $f"
done

command -v sgdisk >/dev/null || die "sgdisk not found (apt install gdisk)"
command -v fw_setenv >/dev/null || die "fw_setenv not found (apt install libubootenv-tool)"

if [ -n "$KERNEL" ] || [ -n "$DTB" ]; then
	[ -n "$KERNEL" ] && [ -r "$KERNEL" ] || die "KERNEL='$KERNEL' unreadable"
	[ -n "$DTB" ]    && [ -r "$DTB" ]    || die "DTB='$DTB' unreadable"
	command -v mkfs.vfat >/dev/null || die "mkfs.vfat not found (apt install dosfstools)"
	command -v mcopy     >/dev/null || die "mcopy not found (apt install mtools)"
fi

[ -b "$DEV" ] || die "$DEV is not a block device"

# refuse to touch a device that isn't the eMMC user area.  eMMC hw boot
# partitions live at ${DEV}boot0 / ${DEV}boot1; if those don't exist the
# device isn't an eMMC and we bail rather than nuke somebody's SD.
[ -b "${DEV}boot0" ] || die "$DEV has no boot0 hw partition; refusing to flash"
[ -b "${DEV}boot1" ] || die "$DEV has no boot1 hw partition; refusing to flash"

# name-only sanity: the SD on this board comes up as mmcblk0.  Guard against
# a typo pointing at it.
case "$DEV" in
*mmcblk0*) die "$DEV looks like the SD card; refusing to flash" ;;
esac

echo "=== will flash $DEV ==="
lsblk "$DEV" || true
[ -n "$KERNEL" ] && echo "  will populate 9fat with KERNEL=$KERNEL DTB=$DTB"
[ -n "$PLAN9INI" ] && echo "  and PLAN9INI=$PLAN9INI"
read -r -p "type YES to proceed: " confirm
[ "$confirm" = YES ] || die "aborted"

echo
echo "=== 1. wipe first 4 MiB of user area and any GPT ==="
dd if=/dev/zero of="$DEV" bs=1M count=4 conv=fsync status=none
sgdisk --zap-all "$DEV" >/dev/null

echo
echo "=== 2. create GPT with named partitions ==="
sgdisk \
	--new=1:2048:4095    --typecode=1:8300 --change-name=1:opensbi \
	--new=2:4096:8191    --typecode=2:8300 --change-name=2:uboot \
	--new=3:768:895      --typecode=3:8300 --change-name=3:env \
	--new=4:16384:212991 --typecode=4:0700 --change-name=4:9fat \
	--new=5:212992:0     --typecode=5:"$PLAN9_TYPE_GUID" --change-name=5:plan9 \
	"$DEV"

# reload partition table (partprobe isn't always available on minimal Bianbu)
if command -v partprobe >/dev/null; then
	partprobe "$DEV"
elif command -v blockdev >/dev/null; then
	blockdev --rereadpt "$DEV"
fi
sleep 1

echo
echo "=== 3. write firmware into named partitions ==="
dd if="$OPENSBI_ITB" of="${DEV}p1" bs=1M    conv=fsync status=none
dd if="$UBOOT_ITB"   of="${DEV}p2" bs=1M    conv=fsync status=none
dd if="$ENV_BIN"     of="${DEV}p3" bs=1K    conv=fsync status=none

echo
echo "=== 4. customize u-boot env (bootdelay, disable NFS lookup, bootcmd) ==="
# fw_setenv wants a config file describing where env lives.  Write a temp
# one so we don't touch /etc/fw_env.config (which may already point at SD).
tmpcfg=$(mktemp)
trap 'rm -f "$tmpcfg"' EXIT
printf '%s 0x0 0x4000\n' "${DEV}p3" >"$tmpcfg"

# Give ourselves an interrupt window and disable the vendor NFS
# mac-mapping lookup that otherwise stalls boot ~20s waiting for a server
# we don't have.
fw_setenv -c "$tmpcfg" bootdelay 3
fw_setenv -c "$tmpcfg" mac_mapping_file ''
fw_setenv -c "$tmpcfg" serverip ''

# bootcmd zeroes CONFADDR and loads plan9.ini over it, then loads the
# 9front kernel + DTB from mmc 2:4 (the 9fat partition), sets a fallback
# /chosen/bootargs for when the file is missing, and hands off via booti.
# 0x50000000 is a staging address in DRAM well clear of u-boot's own
# footprint; booti relocates from there to KTZERO (0x40200000) based on
# the Image header in the kernel binary, leaving CONFADDR untouched.
bootcmd='mw.b 0x40010000 0x0 0x10000;'
bootcmd+=' if load mmc 2:4 0x40010000 plan9.ini; then echo plan9.ini loaded; fi;'
bootcmd+=' load mmc 2:4 0x50000000 9bpif3;'
bootcmd+=' load mmc 2:4 0x48000000 k1-x_deb1.dtb;'
bootcmd+=' fdt addr 0x48000000;'
bootcmd+=' fdt resize;'
bootcmd+=' fdt chosen;'
bootcmd+=" fdt set /chosen bootargs \"$BOOTARGS\";"
bootcmd+=' booti 0x50000000 - 0x48000000'
fw_setenv -c "$tmpcfg" bootcmd "$bootcmd"

echo "customized env now says:"
fw_printenv -c "$tmpcfg" bootdelay bootcmd | sed 's/^/    /'

echo
echo "=== 5. write bootinfo + FSBL to eMMC boot0/boot1 hw partitions ==="
# BootROM on the K1 reads bootinfo from boot0 and FSBL (SPL) from boot1.
# These hw partitions are marked read-only from Linux by default.
echo 0 >"/sys/block/$(basename "$DEV")boot0/force_ro"
echo 0 >"/sys/block/$(basename "$DEV")boot1/force_ro"

dd if="$BOOTINFO" of="${DEV}boot0" bs=512 conv=fsync status=none
dd if="$FSBL"     of="${DEV}boot1" bs=512 conv=fsync status=none

# re-lock the boot partitions
echo 1 >"/sys/block/$(basename "$DEV")boot0/force_ro"
echo 1 >"/sys/block/$(basename "$DEV")boot1/force_ro"

# eMMC BOOT_PART_ENABLE: 0=off, 1=boot0, 2=boot1.  BootROM uses whichever
# BOOT_PART_ENABLE points at; if this ext_csd byte is already right the
# board boots, if not the board hangs.  mmc-utils sets it if available.
if command -v mmc >/dev/null; then
	# arg meaning: mmc bootpart enable <ack> <boot_partition> <device>
	# ack=1, partition 2 = boot1 (SpacemiT convention).  Adjust to 1
	# (boot0) if a future FSBL is flashed there instead.
	mmc bootpart enable 2 1 "$DEV" || echo "warning: 'mmc bootpart enable' failed"
else
	echo "warning: mmc-utils not installed; BOOT_PART_ENABLE not set"
	echo "         run 'apt install mmc-utils' and re-run this script if boot fails"
fi

echo
echo "=== 6. populate 9fat partition ==="
if [ -n "$KERNEL" ]; then
	mkfs.vfat -F 32 -n 9fat "${DEV}p4" >/dev/null
	mcopy -i "${DEV}p4" "$KERNEL" ::9bpif3
	mcopy -i "${DEV}p4" "$DTB"    ::k1-x_deb1.dtb
	tmpini=$(mktemp)
	if [ -n "$PLAN9INI" ] && [ -r "$PLAN9INI" ]; then
		cp "$PLAN9INI" "$tmpini"
	else
		# one key=value per line; $BOOTARGS values contain no spaces
		echo "$BOOTARGS" | tr ' ' '\n' >"$tmpini"
	fi
	mcopy -i "${DEV}p4" "$tmpini" ::plan9.ini
	rm -f "$tmpini"
	echo "9fat contents:"
	mdir -i "${DEV}p4" -/ | sed 's/^/    /'
else
	echo "KERNEL/DTB not provided; 9fat left unformatted."
	echo "Format and populate later with:"
	echo "    mkfs.vfat -F 32 -n 9fat ${DEV}p4"
	echo "    mcopy -i ${DEV}p4 /path/9bpif3 ::9bpif3"
	echo "    mcopy -i ${DEV}p4 /path/k1-x_deb1.dtb ::k1-x_deb1.dtb"
	echo "    mcopy -i ${DEV}p4 /path/plan9.ini ::plan9.ini"
fi

echo
echo "=== 7. verify partition table ==="
sgdisk -p "$DEV"

echo
echo "done."
if [ -n "$KERNEL" ]; then
	echo "power off, remove SD, and power on to boot 9front from eMMC."
else
	echo "populate ${DEV}p4 (9fat) before removing SD, or the board will"
	echo "boot to a u-boot prompt with no kernel to load."
fi
