all:		defconfig-gta01 defconfig-gta02

defconfig-gta01: \
		defconfig-2.6.24-maxmodules
		cp $< $@

defconfig-gta02: \
		defconfig-2.6.24-maxmodules
		sed '/UART.*=0$$/s/=0/=2/' <$< >$@ || { rm -f $@; exit 1; }
