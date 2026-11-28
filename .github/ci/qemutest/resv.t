# regions at a fixed place in /reserved-memory are left out of every
# bank, the kernel's included; reusable pools are RAM, even one over
# the kernel as on K1
MEM=2G
dts() {
	sed -e 's/reg = <0x00 0x80000000 0x00 0x80000000>;/reg = <0x00 0x80000000 0x00 0x40000000>;/' -e '$d'
	cat <<'Y'
	memory@c0000000 {
		device_type = "memory";
		reg = <0x00 0xc0000000 0x00 0x40000000>;
	};
	reserved-memory {
		#address-cells = <0x02>;
		#size-cells = <0x02>;
		ranges;
		low@a0000000 {
			reg = <0x00 0xa0000000 0x00 0x1000000>;
			no-map;
		};
		hole@d0000000 {
			reg = <0x00 0xd0000000 0x00 0x10000000>;
			no-map;
		};
		cma {
			compatible = "shared-dma-pool";
			reusable;
			size = <0x00 0x18000000>;
			alloc-ranges = <0x00 0x80000000 0x00 0x30000000>;
		};
		pool {
			compatible = "shared-dma-pool";
			reusable;
			size = <0x00 0x1000000>;
			alloc-ranges = <0x00 0xf0000000 0x00 0x8000000>;
		};
	};
};
Y
}
script() {
	cat <<'X'
want "meminit: bank 0x80800000-0xa0000000"
want "meminit: bank 0xa1000000-0xd0000000"
want "meminit: bank 0xe0000000-0x100000000"
rc
X
}
