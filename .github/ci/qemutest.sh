#!/bin/sh
# qemutest.sh [-o dir] KERNEL [test ...]
# Boots a riscv64 kernel under qemu once per test in qemutest/*.t
# (or the named ones, which may be in subdirectories: maxion/smoke) and drives its console with expect.
#
# A test is a sh fragment that may set
#	MACHINE	qemu -M value (default virt)
#	SMP MEM CPU	-smp, -m, -cpu
#	QEMUARGS	more qemu arguments
#	APPEND	kernel command line (default console=0)
# and may define dts(), a filter from the machine's own device tree
# source to the one booted.  It must define script(), whose output is
# expect commands using the procs in qemutest.exp.
# Logs go to dir (default qemutest.out), one NAME.log per test.

out=qemutest.out
while getopts o: c; do
	case $c in
	o) out=$OPTARG ;;
	*) echo usage: $0 '[-o dir] KERNEL [test ...]' >&2; exit 2 ;;
	esac
done
shift $((OPTIND-1))
test $# -ge 1 || { echo usage: $0 '[-o dir] KERNEL [test ...]' >&2; exit 2; }
kernel=$(realpath "$1"); shift
ci=$(dirname "$(realpath "$0")")
mkdir -p "$out"; out=$(realpath "$out")
test $# -eq 0 && set -- $(cd "$ci/qemutest" && ls *.t | sed 's/\.t$//')

pass=0; failed=
for t in "$@"; do
	t=${t%.t}
	(
		MACHINE=virt SMP=1 MEM=1G CPU=rv64,svpbmt=true,zicbom=true,sstc=true
		QEMUARGS= APPEND=console=0
		. "$ci/qemutest/$t.t"
		q="qemu-system-riscv64 -M $MACHINE -smp $SMP -m $MEM -cpu $CPU"
		dtb=
		mkdir -p "$(dirname "$out/$t")"
		if command -v dts >/dev/null; then
			$q -display none -machine dumpdtb="$out/$t.base.dtb" >/dev/null 2>&1 || exit 1
			dtc -q -I dtb -O dts "$out/$t.base.dtb" | dts >"$out/$t.dts" &&
			dtc -q -I dts -O dtb -o "$out/$t.dtb" "$out/$t.dts" || exit 1
			dtb="-dtb $out/$t.dtb"
		fi
		export QEMU="$q -bios default -nodefaults -nographic -monitor none -serial stdio $dtb $QEMUARGS -kernel $kernel"
		export APPEND
		{ cat "$ci/qemutest.exp"; script; echo pass; } >"$out/$t.exp"
		expect -f "$out/$t.exp" >"$out/$t.log" 2>&1 </dev/null
	)
	if [ $? -eq 0 ]; then
		echo "PASS $t"; pass=$((pass+1))
	else
		echo "FAIL $t: $(tail -1 "$out/$t.log" 2>/dev/null)"; failed="$failed $t"
	fi
done
echo "$pass passed${failed:+, failed:$failed}"
test -z "$failed"
