# no PCIe host in the DT: no PCI devices
dts() {
	sed '/pci@30000000 {/,/^\t\t};/d'
}
script() {
	cat <<'X'
rc
lacks {ls '#$/pci'} "ctl"
X
}
