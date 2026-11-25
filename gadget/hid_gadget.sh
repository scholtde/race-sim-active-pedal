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
echo 0x0100 > bcdDevice
echo 0x0200 > bcdUSB
mkdir -p strings/0x409
echo "0001"                > strings/0x409/serialnumber
echo "race-sim-active-pedal" > strings/0x409/manufacturer
echo "Active Pedal"        > strings/0x409/product

mkdir -p functions/hid.usb0
echo 0 > functions/hid.usb0/protocol
echo 0 > functions/hid.usb0/subclass
echo 5 > functions/hid.usb0/report_length
printf '\x05\x01\x09\x04\xa1\x01'\
'\x05\x09\x19\x01\x29\x08\x15\x00\x25\x01\x75\x01\x95\x08\x81\x02'\
'\x05\x01\x09\x01\xa1\x00\x09\x30\x09\x31\x15\x00\x26\xff\x0f\x75\x10\x95\x02\x81\x02\xc0'\
'\xc0' > functions/hid.usb0/report_desc

mkdir -p configs/c.1/strings/0x409
echo "pedal" > configs/c.1/strings/0x409/configuration
echo 100 > configs/c.1/MaxPower
ln -s functions/hid.usb0 configs/c.1/

ls /sys/class/udc | head -1 > UDC
chmod 666 /dev/hidg0 2>/dev/null || true
echo "gadget up on $(cat UDC)"
