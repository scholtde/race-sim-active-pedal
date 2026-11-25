"""Run the pedal (brake + throttle on A2): monitor.py <seconds> [setup cmds...], e.g. monitor.py 120 V400 I10.
"@<sec>:<cmd>" sends a command mid-run, e.g. @50:L44.
Calibrates and arms, prints live force/position, always disarms on exit (incl. Ctrl-C)."""
import serial, time, sys
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.3)
time.sleep(2.2); s.reset_input_buffer()
# setup cmds; "@<sec>:<cmd>" sends <cmd> that many seconds after start instead
timed = sorted((float(c[1:].split(":")[0]), c.split(":", 1)[1]) for c in sys.argv[2:] if c.startswith("@"))
for c in (sys.argv[2:] or []):
    if c.startswith("@"): continue
    s.write((c + "\n").encode()); time.sleep(0.25)
s.write(b"g\n")                                   # arm (calibrates first)
t0, last_print, peak, events = time.time(), 0, 0.0, []
thr_max, both = 0.0, 0.0                          # throttle peak; secs of brake+throttle together
prev_mode = None
raw = open("monitor_raw.txt", "w")                # every line, for analysis
try:
    while time.time() - t0 < secs:
        while timed and time.time() - t0 >= timed[0][0]:
            s.write((timed.pop(0)[1] + "\n").encode())
        ln = s.readline().decode("ascii", "replace").strip()
        if not ln: continue
        raw.write("%.3f %s\n" % (time.time()-t0, ln))
        if not ln.startswith("S,"):
            print("      %s" % ln); continue
        p = ln.split(",")
        if len(p) < 5: continue
        try: kg, sp, mA, mode = float(p[1]), int(p[2]), int(p[3]), p[4]
        except ValueError: continue
        mm = p[5] if len(p) > 5 else "?"
        thr = float(p[12]) if len(p) > 12 else 0.0       # throttle %, last field
        peak = max(peak, kg); thr_max = max(thr_max, thr)
        if thr > 5 and kg > 2: both += 0.05              # S lines come every 50 ms
        if mode != prev_mode:
            events.append((time.time()-t0, prev_mode, mode, kg))
            prev_mode = mode
        now = time.time()
        if now - last_print >= 0.2:
            last_print = now
            bb = "#" * max(0, min(20, int(kg/25*20)))     # brake bar: 25 kg full
            tb = "=" * max(0, min(20, int(thr/100*20)))   # throttle bar: 100 %
            print("%5.1f  BRK %6.2f kg |%-20s|  THR %5.1f %% |%-20s|  %6s mm %5dmA  %s"
                  % (now-t0, kg, bb, thr, tb, mm, mA, mode))
except KeyboardInterrupt:
    print("\nCtrl-C")
finally:
    s.write(b"x\n"); time.sleep(0.2); s.close(); raw.close()   # always disarm

print("\npeak force: %.2f kg   peak throttle: %.1f %%   brake+throttle together: %.1f s" % (peak, thr_max, both))
print("mode transitions:")
for t, a, b, kg in events:
    print("   %5.1fs  %-9s -> %-9s at %.1f kg" % (t, a, b, kg))
