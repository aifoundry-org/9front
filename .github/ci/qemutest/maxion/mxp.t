# 9maxion with the host's packet mailbox bootstrap and time sample in
# the device tree.  The page is in qemu's PCIe ECAM, which reads no
# firmware, so the link stops at the attach check once ipconfig has
# bound it and the address cannot be added.
CPU=rv64,svpbmt=false,zicbom=false,sstc=false
APPEND='console=0 user=glenda *soc=maxion *ncpu=1 *uartpoll=1'
dts() {
	sed -e '/chosen {/a\
		esperanto,mxpacket-page-initialized;\
		esperanto,mxpacket-abi = <1>;\
		esperanto,mxpacket-fw-id = <1 2 3 4>;\
		ainekko,rtc-sample = <0 0 0x17979cfe 0x362a0000>;' \
	-e '$i\
	packet@30001000 {\
		compatible = "esperanto,maxion-packet-v1";\
		reg = <0 0x30001000 0 0x1000>;\
	};'
}
script() {
	cat <<'X'
want "rtc: host sample 1700000000.000000000"
rc
run {bind -a '#I' /net}
lacks {ip/ipconfig netdev '#E/mxp' 192.0.2.2 255.255.255.255 192.0.2.1 >[2=1]} "bind netdev"
has {cat '#E/mxpctl'} "stopped attach"
X
}
