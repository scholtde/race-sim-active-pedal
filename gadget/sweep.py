#!/usr/bin/env python3
"""Test the gadget: sweep brake and throttle so you can watch them in Windows
(Win+R -> joy.cpl -> Active Pedal -> Properties). Ctrl-C to stop."""
import math, struct, time
with open("/dev/hidg0", "wb", buffering=0) as f:
    t0 = time.time()
    while True:
        t = time.time() - t0
        brake = int(2047.5 + 2047.5 * math.sin(t * 2.0))
        thr = int(2047.5 + 2047.5 * math.sin(t * 2.0 + math.pi))
        btn = 1 if int(t) % 2 else 0
        try:
            f.write(struct.pack("<BHH", btn, brake, thr))
        except (BlockingIOError, BrokenPipeError, OSError):
            time.sleep(0.1)  # host not listening yet
        time.sleep(0.004)
