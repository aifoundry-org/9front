# APLIC source modes come from the DT's interrupt flags
MACHINE=virt,aia=aplic-imsic
dts() {
	sed '/serial@10000000 {/,/};/s/interrupts = <0x0a 0x04>;/interrupts = <0x0a 0x01>;/'
}
script() {
	cat <<'X'
want "aplic: src 10 for eia32: cfg 4 "
X
}
