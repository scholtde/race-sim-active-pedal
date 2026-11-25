#!/bin/sh
# Create (or remove with "down") the USB HID game controller gadget.
# Needs root, a USB device controller (/sys/class/udc) and libcomposite.
# Report (5 bytes): buttons(8 bits), X = brake (u16 0..4095), Y = throttle (u16 0..4095)
set -e
G=/sys/kernel/config/usb_gadget/pedal

down() {
  [ -d $G ] || return 0
  echo "" > $G/UDC 2>/dev/null || true
  rm -f $G/configs/c.1/hid.usb0
  rmdir $G/configs/c.1/strings/0x409 $G/configs/c.1 2>/dev/null || true
  rmdir $G/functions/hid.usb0 $G/strings/0x409 $G 2>/dev/null || true
}

if [ "$1" = down ]; then down; exit 0; fi

modprobe libcomposite
mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config
down

mkdir -p $G && cd $G
echo 0x1d6b > idVendor          # Linux Foundation
echo 0x0104 > idProduct         # multifunction composite gadget
echo 0x0101 > bcdDevice        # bump when the descriptor changes (Windows caches it)
echo 0x0200 > bcdUSB
mkdir -p strings/0x409
echo "0001"                > strings/0x409/serialnumber
echo "race-sim-active-pedal" > strings/0x409/manufacturer
echo "Active Pedal"        > strings/0x409/product

mkdir -p functions/hid.usb0
echo 0 > functions/hid.usb0/protocol
echo 0 > functions/hid.usb0/subclass
echo 5 > functions/hid.usb0/report_length
# octal escapes: dash's printf (Ubuntu /bin/sh) does not understand \x
printf '\005\001\011\004\241\001\005\011\031\001\051\010\025\000\045\001\165\001\225\010\201\002\005\001\011\001\241\000\011\060\011\061\025\000\046\377\017\165\020\225\002\201\002\300\300' > functions/hid.usb0/report_desc

mkdir -p configs/c.1/strings/0x409
echo "pedal" > configs/c.1/strings/0x409/configuration
echo 100 > configs/c.1/MaxPower
ln -s functions/hid.usb0 configs/c.1/

ls /sys/class/udc | head -1 > UDC
chmod 666 /dev/hidg0 2>/dev/null || true
echo "gadget up on $(cat UDC)"
