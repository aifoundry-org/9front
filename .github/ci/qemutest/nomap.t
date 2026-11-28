# a pool that must not be mapped is left out, like the BeagleV-Fire's
# uncached aliases of RAM; disabled nodes don't count
MEM=2G
dts() {
	sed -e 's/reg = <0x00 0x80000000 0x00 0x80000000>;/reg = <0x00 0x80000000 0x00 0x40000000>;/' -e '$d'
	cat <<'Y'
	memory@c0000000 {
		device_type = "memory";
		reg = <0x00 0xc0000000 0x00 0x40000000>;
	};
	memory@200000000 {
		device_type = "memory";
		reg = <0x02 0x00 0x00 0x40000000>;
		status = "disabled";
	};
	reserved-memory {
		#address-cells = <0x02>;
		#size-cells = <0x02>;
		ranges;
		uncached {
			compatible = "shared-dma-pool";
			size = <0x00 0x10000000>;
			no-map;
			alloc-ranges = <0x00 0xe0000000 0x00 0x10000000>;
		};
		off@c0000000 {
			reg = <0x00 0xc0000000 0x00 0x10000000>;
			no-map;
			status = "disabled";
		};
	};
};
Y
}
script() {
	cat <<'X'
want "meminit: uzero 0x100000000"
want "meminit: bank 0x80800000-0xe0000000"
want "meminit: bank 0xf0000000-0x100000000"
rc
has {awk '$2 == "memory" { if($1 > 1800000000 && $1 <= 1879048192) print "memory ok"; else print "memory " $1 }' /dev/swap} "memory ok"
X
}
