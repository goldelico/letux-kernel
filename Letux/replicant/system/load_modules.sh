#!/system/bin/sh
#Debian/4.5 uses this extra modules, which are unused in Replicant, up to now:
#autofs4                25916  1
#usb_f_ecm               7039  1
#g_ether                 4993  0
#usb_f_rndis            16962  2 g_ether
#u_ether                13270  3 usb_f_ecm,usb_f_rndis,g_ether
#ipv6                  410330  20
#hso                    30144  0 #(this is built-in the Replicant kernel)
#twl4030_madc_hwmon      3361  0
#twl4030_madc_battery     3998  0

#This scipt loads some modules, which are not loaded automatically (for some reason)
DEVICE=$(cat /sys/firmware/devicetree/base/model)
echo "Detected model:" $DEVICE

#Graphics
modprobe panel_tpo_td028ttec1 #Letux 2804
modprobe panel_dpi #Letux 3704/7004
modprobe omapdrm
modprobe omapdss
modprobe connector-analog-tv
modprobe encoder-opa362

#Audio
modprobe snd-soc-twl4030
modprobe snd-soc-simple-card
modprobe snd-soc-omap-twl4030
modprobe snd-soc-omap-mcbsp

#Backlight
modprobe pwm-omap-dmtimer
modprobe pwm_bl
chown system.system /sys/class/backlight/backlight/brightness
chown system.system /sys/class/backlight/backlight/max_brightness

#Touchscreen
modprobe tsc2007

#Battery/Charger
modprobe phy-twl4030-usb
modprobe twl4030_madc
modprobe twl4030_charger allow_usb=1
modprobe omap_hdq
#modprobe w1_bq27000
case "$DEVICE" in
    "Goldelico GTA04A3/Letux 2804" | "Goldelico GTA04A4/Letux 2804" | "Goldelico GTA04A5/Letux 2804" )
        #GTA04 a3/a4/a5 (Letux 2804)
        echo "LOADING bq27xxx_battery"
        modprobe bq27xxx_battery
        echo 5 > /sys/module/bq27xxx_battery/parameters/poll_interval
        ;;
    "Goldelico GTA04b2/Letux 3704" | "Goldelico GTA04b3/Letux 7004" )
        #GTA04 b2/b3 (Letux 3704 / Letux 7004)
        echo "LOADING generic_adc_battery"
        modprobe generic_adc_battery
        ;;
    * ) #Fallback
        echo "Could not detect device variant."
        echo "Check your device-tree model (/sys/firmware/devicetree/base/model = $DEVICE)."
        echo "Falling back to bq27xxx_battery"
        modprobe bq27xxx_battery
        echo 5 > /sys/module/bq27xxx_battery/parameters/poll_interval
        ;;
esac

#Power Button
modprobe twl4030-pwrbutton

#USB FunctionFS (ADB)
modprobe omap2430 #used for MUSB/UDC
modprobe libcomposite
modprobe usb_f_fs
#modprobe g_ffs idVendor=0x18d1 idProduct=0x4e26 #FIXME: Busybox modprobe does not correctly pass the module parameters
insmod /system/lib/modules/g_ffs.ko idVendor=0x18d1 idProduct=0x4e26
#Temoporary ADB fix:
mount -o uid=2000,gid=2000 -t functionfs adb /dev/usb-ffs/adb
restart adbd

#Vibracall
modprobe twl4030_vibra

#Bluetooth
modprobe w2cbw003-bluetooth
modprobe hci_uart

#WiFi
modprobe leds-tca6507 #needed for MMC reset/power (the WiFi chip is connected via MMC)
modprobe libertas #autoloads the cfg80211 dependency
#libertas_sdio is loaded by the Android framework, once WiFi is enabled
modprobe wl18xx
modprobe wlcore_sdio

#WWAN
modprobe ehci-omap
modprobe wwan-on-off
chmod 777 /dev/rfkill

#Sensors
modprobe bmp280-i2c
modprobe itg3200
modprobe hmc5843_i2c
modprobe lis3lv02d_i2c
modprobe bma180
chmod 666 /dev/input/*
#chmod 666 /sys/class/input/*/poll
chmod 666 /dev/iio:device*
#chmod 666 /sys/bus/iio/devices/iio:device*/*
chmod 666 /sys/bus/iio/devices/iio:device*/scan_elements/*
chmod 666 /sys/bus/iio/devices/iio:device*/trigger/*
chmod 666 /sys/bus/iio/devices/iio:device*/buffer/enable
chmod 666 /sys/bus/iio/devices/iio:device*/sampling_frequency

#GPS
modprobe w2sg0004
modprobe extcon-gpio

#Misc
modprobe gpio_twl4030
#modprobe rtc_twl
modprobe at24 #I2C-EEPROM
sleep 6
kill `pidof mediaserver`
