# the kernel sits 256MiB into RAM; RAM below it is used too, less the
# firmware's own reserved-memory nodes.  The kernel's share covers
# only part of it, so user pages and DMA buffers come from there.
LOADADDR=0x90200000
APPEND="console=0 *kernelpercent=10"
QEMUARGS="-netdev user,id=n0 -device virtio-net-pci,netdev=n0,disable-legacy=on,romfile="
script() {
	cat <<'X'
wantnot "bank 0x80000000-" "-0x90000000"
want "meminit: bank 0x90800000-0xc0000000"
rc
has {awk '$2 == "memory" { if($1 > 1060000000 && $1 <= 1073741824) print "memory ok"; else print "memory " $1 }' /dev/swap} "memory ok"
has {awk 'BEGIN { s = "0123456789abcdef"; for(i = 0; i < 20; i++) s = s s; print length(s), gsub(/0123456789abcdef/, "", s) }'} "16777216 1048576"
run {bind -a '#I' /net; bind -a '#l0' /net}
run {ip/ipconfig ether /net/ether0}
has {cat /net/ndb} "ip=10.0.2.15"
X
}
