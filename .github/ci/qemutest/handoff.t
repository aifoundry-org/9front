# SBI cold-boots hart 2, which hands the boot to hart 0; all harts
# come up.  OpenSBI 1.7 honours cold-boot-harts, QEMU 8.2's doesn't.
SMP=4
QEMUARGS="-bios /usr/lib/riscv64-linux-gnu/opensbi/generic/fw_dynamic.bin"
dts() {
	sed 's/^\t\tcpu@2 {/\t\tcpu2: cpu@2 {/'
	cat <<'X'
/ {
	chosen {
		opensbi-config {
			compatible = "opensbi,config";
			cold-boot-harts = <&cpu2>;
		};
	};
};
X
}
script() {
	cat <<'X'
want "Boot HART ID"
wantnot "\n" ": 2\r"
want "hartid 0 dtb"
rc
has {cat /dev/sysstat} "          0 " "          3 "
X
}
