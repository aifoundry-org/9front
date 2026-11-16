#!/bin/sh
# smoke.sh ARCH KERNEL ARCH.tar.gz, run from the top of a fresh checkout.
# Boots KERNEL on a root made of the checkout plus ARCH's binaries and
# runs glenda's smoke test from cpurc's last hook, after the network is up.
set -e

tar -xf "$3"
mkdir -p tmp cfg/9
echo '/usr/glenda/bin/rc/smoke && echo BATCH-PASS || echo BATCH-FAIL' >cfg/9/cpustart

SERVICE=cpu KERNEL=$1/$2 expect <<EOF
spawn boot/qemu -$1
expect -timeout -1 BATCH-PASS {exit 0} BATCH-FAIL {exit 1} eof {exit 1}
EOF
