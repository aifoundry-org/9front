# no interrupt controller in the DT: no guessed address, said so
dts() {
	sed '/plic@c000000 {/,/^\t\t};/d'
}
script() {
	cat <<'X'
wantnot "plic:" "intrinit: no interrupt controller"
X
}
