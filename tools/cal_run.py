"""Run one limit-switch calibration: send 'c', print every C/STOP line and a
throttled E line, stop at done/abort or the timeout. Sends 'x' on exit."""
import serial, sys, time
send_cal = "--go" in sys.argv
s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.2)
time.sleep(2.2); s.reset_input_buffer()
if send_cal:
    s.write(b"c\n")
t0 = time.time(); lastE = 0
try:
    while time.time() - t0 < (45 if send_cal else 3):
        ln = s.readline().decode("ascii", "replace").strip()
        if not ln: continue
        now = time.time() - t0
        if ln.startswith(("E,", "S,")):
            if now - lastE < 0.3: continue
            lastE = now
        print("%5.1f  %s" % (now, ln), flush=True)
        if "calibration done" in ln:
            t_end = time.time()
            while time.time() - t_end < 1.5:
                l2 = s.readline().decode("ascii", "replace").strip()
                if l2.startswith("C,"): print("       %s" % l2, flush=True)
            break
        if "ABORT" in ln: break
finally:
    s.write(b"x\n"); time.sleep(0.2); s.close()
