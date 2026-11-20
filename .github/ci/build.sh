#!/bin/sh
# build.sh ARCH AMD64.tar.gz, run from the top of a fresh checkout.
# Rebuilds ARCH's userland and kernels in an amd64 guest whose root is the
# checkout plus the amd64 binaries, and packs the result as ARCH.tar.gz.
set -e

# Make the checkout a root: binaries, the empty directories git cannot
# track, a time zone, and a network database whose entry for the
# launcher's NIC makes cpurc configure it.
tar -xf "$2"
sh sys/lib/rootstub
cp -R sys/lib/dist/ndb/. lib/ndb
echo 'sys=9 dom=front ether=525400123456' >>lib/ndb/local
cp adm/timezone/US_Pacific adm/timezone/local

# Booted as a cpu server, the guest runs buildarch from cpurc's last
# hook, /cfg/$sysname/cpustart; the ndb entry above names it 9.
mkdir -p cfg/9
echo "/usr/glenda/bin/rc/buildarch $1 && echo BATCH-PASS || echo BATCH-FAIL" >cfg/9/cpustart

# The launcher's default of one CPU matters: build runs a mk job per CPU
# and concurrent jobs stall the 9P root.
SERVICE=cpu expect <<EOF
spawn boot/qemu -amd64
expect -timeout -1 BATCH-PASS {exit 0} BATCH-FAIL {exit 1} eof {exit 1}
EOF

# include and mkfile are tracked sources, not products.
tar -czf "$1.tar.gz" --exclude="$1/include" --exclude="$1/mkfile" \
	"$1" "acme/bin/$1" lib/ndb adm/timezone/local
