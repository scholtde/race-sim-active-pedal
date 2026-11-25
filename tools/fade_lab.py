"""SIDE EXPERIMENT (not in the firmware): pedal fade driven from the Pi.

  fade_lab.py abs <secs> [setup cmds...]   ABS = one-way knocks + pedal collapsing toward the stop
  fade_lab.py hot <secs> [setup cmds...]   no ABS; brakes heat with use, the pedal goes long and soft

The fade is done by re-sending p (full-travel force) and B (bite point) live while you
press. Opening the port resets the Uno, so nothing here sticks: the next monitor.py run
is back to normal. Setup cmds work as in monitor.py (p and B are read from them).
Tunables: env vars, e.g.  FADE_S=1.0 fade_lab.py abs 90 ...   (see the TUNE dict below).
"""
import serial, time, sys, os

mode = sys.argv[1] if len(sys.argv) > 1 else ""
if mode not in ("abs", "hot"):
    print(__doc__); sys.exit(1)
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
setup = sys.argv[3:]

TUNE = {  # name: default
    "FADE_S":   1.2,    # abs: seconds for the pedal to collapse to the floor
    "FLOOR":    0.85,   # abs: p falls to FLOOR * (force at ABS entry) -> pedal reaches the stop
    "BACK_S":   0.5,    # abs: seconds to firm back up after ABS lets go
    "HEAT_TAU": 30.0,   # hot: cooling time constant, s
    "HEAT_ON":  60.0,   # hot: heat at which fade starts   (8 kg held ~9 s)
    "HEAT_FULL":140.0,  # hot: heat at full fade            (8 kg held ~25 s)
    "LOCK_KG":  14.0,   # hot: above this the wheels lock -> no more heat goes in
    "SOFT":     0.5,    # hot: full fade cuts p by this fraction (softer)
    "LONG_MM":  10.0,   # hot: full fade moves the bite this much deeper (long pedal)
}
T = {k: float(os.environ.get(k, v)) for k, v in TUNE.items()}

def arg(prefix, default):
    for c in setup:
        if c.startswith(prefix) and not c.startswith("@"):
            return c[len(prefix):]
    return default
p0 = float(arg("p", "20"))
bmm, bkg = (float(x) for x in arg("B", "22,2").split(","))
p_min = bkg + 0.7            # firmware drops the bite curve if p < bite_kg + 0.5

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.3)
time.sleep(2.2); s.reset_input_buffer()
def send(c):
    s.write((c + "\n").encode())
for c in setup:
    if not c.startswith("@"):
        send(c); time.sleep(0.25)
if mode == "abs":
    send("b"); time.sleep(0.25)          # one-way pulses: knocks that push the pedal DEEPER
else:
    send("a99"); time.sleep(0.25)        # race car: no ABS
send("g")                                # arm (calibrates first)

raw = open("fade_raw.txt", "w")
t0 = time.time(); last_print = last_send = 0.0
p_sent, b_sent = p0, bmm
abs_t = None; abs_floor = p0; rel_t = None; rel_from = p0
heat, last_t, peak_heat = 0.0, None, 0.0
quiet = ("full travel", "pad bite")      # don't echo our own live updates
try:
    while time.time() - t0 < secs:
        ln = s.readline().decode("ascii", "replace").strip()
        if not ln: continue
        now = time.time(); t = now - t0
        raw.write("%.3f %s\n" % (t, ln))
        if not ln.startswith("S,"):
            if not ln.startswith(quiet): print("      %s" % ln)
            continue
        f = ln.split(",")
        try: kg, mA, pm, mm = float(f[1]), int(f[3]), f[4], f[5]
        except (ValueError, IndexError): continue
        dt = 0.05 if last_t is None else min(0.2, now - last_t); last_t = now

        p_want, b_want, info = p0, bmm, ""
        if mode == "abs":
            in_abs = pm.startswith("ABS")
            if in_abs:
                if abs_t is None:
                    abs_t, rel_t = now, None
                    abs_floor = max(p_min, T["FLOOR"] * kg)
                u = min(1.0, (now - abs_t) / T["FADE_S"])
                p_want = p0 - (p0 - abs_floor) * u
            else:
                if abs_t is not None:
                    abs_t, rel_t, rel_from = None, now, p_sent
                if rel_t is not None:
                    u = min(1.0, (now - rel_t) / T["BACK_S"])
                    p_want = rel_from + (p0 - rel_from) * u
                    if u >= 1: rel_t = None
            info = "p %5.1f" % p_want
        else:
            if bkg < kg < T["LOCK_KG"]:
                heat += kg * dt                  # braking energy goes into the discs
            heat -= heat * dt / T["HEAT_TAU"]    # always cooling
            peak_heat = max(peak_heat, heat)
            fade = min(1.0, max(0.0, (heat - T["HEAT_ON"]) / (T["HEAT_FULL"] - T["HEAT_ON"])))
            p_want = max(p_min, p0 * (1 - T["SOFT"] * fade))
            b_want = bmm + T["LONG_MM"] * fade
            hb = "*" * int(min(1.0, heat / T["HEAT_FULL"]) * 10)
            info = "heat %5.0f |%-10s| fade %3.0f%%%s" % (heat, hb, fade * 100,
                    "  LOCKED" if kg >= T["LOCK_KG"] else "")

        if now - last_send >= 0.1:
            if abs(p_want - p_sent) >= 0.2:
                send("p%.2f" % p_want); p_sent = p_want; last_send = now
            elif abs(b_want - b_sent) >= 0.3:
                send("B%.1f,%.2f" % (b_want, bkg)); b_sent = b_want; last_send = now

        if now - last_print >= 0.2:
            last_print = now
            bb = "#" * max(0, min(20, int(kg / 25 * 20)))
            print("%5.1f  BRK %6.2f kg |%-20s| %6s mm %5dmA  %-7s %s" % (t, kg, bb, mm, mA, pm, info))
except KeyboardInterrupt:
    print("\nCtrl-C")
finally:
    send("x"); time.sleep(0.2); s.close(); raw.close()   # always disarm
if mode == "hot":
    print("\npeak heat %.0f (fade starts %.0f, full %.0f)" % (peak_heat, T["HEAT_ON"], T["HEAT_FULL"]))
