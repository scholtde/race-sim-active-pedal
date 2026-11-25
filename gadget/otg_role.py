#!/usr/bin/env python3
"""Switch the Intel Cherry Trail (UP-CHT01) OTG port between host and device.

usage: sudo otg_role.py device|host

The board firmware forces the port to host mode and kernels before 4.17 have
no role-switch driver. This writes the same bits as Linux's
intel-xhci-usb-role-switch: DUAL_ROLE_CFG0 (vendor xHCI ext cap 192, +0x68)
SW_IDPIN_EN | SW_IDPIN | SW_VBUS_VALID for device, then waits for CFG1
HOST_MODE to follow. Run it before (or after) gadget/hid_gadget.sh.
"""
import mmap, os, struct, sys, time
f = os.open("/sys/bus/pci/devices/0000:00:14.0/resource0", os.O_RDWR | os.O_SYNC)
m = mmap.mmap(f, 0x10000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
r = lambda o: struct.unpack("<I", m[o:o+4])[0]
def w(o, v): m[o:o+4] = struct.pack("<I", v)
off = (r(0x10) >> 16) << 2
while True:
    v = r(off)
    if v & 0xff == 192: break
    off += ((v >> 8) & 0xff) << 2
c0 = off + 0x68
v = r(c0) | (1 << 21)
if sys.argv[1] == "device": v |= (1 << 20) | (1 << 24)
else: v &= ~((1 << 20) | (1 << 24))
w(c0, v)
for _ in range(50):
    if (r(c0 + 4) >> 29 & 1) == (sys.argv[1] != "device"): break
    time.sleep(0.01)
print("CFG0", hex(r(c0)), "HOST_MODE", r(c0 + 4) >> 29 & 1)
