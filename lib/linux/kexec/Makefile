# install gcc on the BPI-F3 and build this there

kexec: kexec.c fdt-bpif3.c
	gcc -o kexec -static ./kexec.c 

fdt-bpif3.c:
	dtc -I fs -O dtb -o bpif3.dtb /proc/device-tree
	fdtput -t s bpif3.dtb /chosen bootargs "$(fdtget bpif3.dtb /chosen bootargs) *maxmem=0x80000000"
	xxd -i bpif3.dtb > fdt-bpif3.c

