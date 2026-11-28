# 9maxion on a hart without Svpbmt, Zicbom or Sstc, as the Maxion's,
# with no packet page or time sample in the device tree
CPU=rv64,svpbmt=false,zicbom=false,sstc=false
APPEND='console=0 user=glenda *soc=maxion *ncpu=1 *uartpoll=1'
script() {
	cat <<'X'
want "maxion: Sv48 active"
want "rtc: no host time sample"
rc
has {cat '#r/rtcctl'} "cold reset"
lacks {ls '#E'} mxpctl
X
}
