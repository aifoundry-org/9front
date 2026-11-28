# DMA reaches only the 1:1 dma-ranges window holding the kernel, so
# the kernel's memory stays below its end; offset windows, windows
# elsewhere such as K1's for its RCPU, buses with other cell sizes
# and disabled buses do not count
MEM=2G
dts() {
	sed '$d'
	cat <<'Y'
	dmabus {
		compatible = "simple-bus";
		#address-cells = <0x02>;
		#size-cells = <0x02>;
		ranges;
		dma-ranges = <0x00 0x00 0x00 0x00 0x00 0x90000000 0x00 0x90000000 0x01 0x00 0x00 0x10000000>;
	};
	rcpubus {
		compatible = "simple-bus";
		#address-cells = <0x02>;
		#size-cells = <0x02>;
		ranges;
		dma-ranges = <0x00 0x30200000 0x00 0x300000 0x00 0xfc000>;
	};
	narrowbus {
		compatible = "simple-bus";
		#address-cells = <0x01>;
		#size-cells = <0x01>;
		dma-ranges = <0x00 0x00 0x00 0x00 0x00 0x88000000 0x00 0x00 0x00 0x00 0x00 0x00>;
	};
	offbus {
		compatible = "simple-bus";
		#address-cells = <0x02>;
		#size-cells = <0x02>;
		ranges;
		dma-ranges = <0x00 0x00 0x00 0x00 0x00 0x88000000>;
		status = "disabled";
	};
};
Y
}
script() {
	cat <<'X'
want "meminit: dma limit 0x90000000"
rc
has {awk 'NF == 2 && $2 == "kernel" { if($1 > 50000 && $1 <= 65536) print "kernel ok"; else print "kernel " $1 }' /dev/swap} "kernel ok"
has {awk '$2 == "memory" { if($1 > 2000000000 && $1 <= 2147483648) print "memory ok"; else print "memory " $1 }' /dev/swap} "memory ok"
X
}
