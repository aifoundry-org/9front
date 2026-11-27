# K1/K3 nodes that are disabled, or on the wrong SoC, attach nothing
dts() {
	cat
	cat <<'X'
/ {
	fake {
		#address-cells = <0x02>;
		#size-cells = <0x02>;
		compatible = "simple-bus";
		ranges;
		mmc@d4281000 { compatible = "spacemit,k1-sdhci"; reg = <0x0 0xd4281000 0x0 0x200>; status = "disabled"; };
		usb@c0a00000 { compatible = "spacemit,k1-dwc3"; reg = <0x0 0xc0a00000 0x0 0x10000>; status = "disabled"; };
		usb@81400000 { compatible = "spacemit,k3-dwc3"; reg = <0x0 0x81400000 0x0 0x10000>; phy-names = "usb2-phy", "usb3-phy"; status = "disabled"; };
		i2c@d4010800 { compatible = "spacemit,k1-i2c"; reg = <0x0 0xd4010800 0x0 0x38>; };
	};
};
X
}
script() {
	cat <<'X'
rc
has {cat '#P/archctl'} "soc none"
lacks {ls '#S'} "sdM" "sdN"
lacks {ls '#J'} "i2c"
lacks {ls '#u'} "ep"
X
}
