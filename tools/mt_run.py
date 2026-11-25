"""Calibrate, then run the motor characterisation ('M') and print its M lines."""
import serial, time
s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.2)
time.sleep(2.2); s.reset_input_buffer()
def until(marker, secs, show=("M,", "STOP", "ABORT")):
    t0 = time.time()
    while time.time() - t0 < secs:
        ln = s.readline().decode("ascii", "replace").strip()
        if ln.startswith(show) or "ABORT" in ln: print(ln, flush=True)
        if marker in ln: return True
        if "ABORT" in ln: return False
    print("timeout waiting for", marker); return False
try:
    s.write(b"c\n")
    if until("calibration done", 30, show=("STOP",)):
        s.write(b"M\n"); until("M,=== done", 90)
finally:
    s.write(b"x\n"); time.sleep(0.2); s.close()
