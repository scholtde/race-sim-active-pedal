#!/usr/bin/env python3
"""pedald - the active pedal's Pi-side daemon.

Owns the Uno's serial port, pushes the active profile to it (again after every Uno
reset), runs the Pi-side effects (ABS collapse, hot-brake fade) and serves the web
GUI on http://<pi>:8080.  Standard library + pyserial only (must run on a Pi Zero).

    python3 pedald.py [--port /dev/ttyACM0] [--http 8080]
"""
import argparse, json, os, queue, re, signal, struct, sys, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import serial

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES = os.path.join(HERE, "profiles")
LOGS = os.path.join(HERE, "logs")
STATE = os.path.join(HERE, "state.json")

# ---------------------------------------------------------------------------
# Knobs. 'cmd' is the firmware command; firmware knobs are sent in this order
# (U before V: the V cap depends on U; d before p: p is checked against d).
# Pi-side knobs (fx.*) never go to the Uno.
KNOBS = [
  # key        group      label                         min   max   step  default  unit   cmd
  ("U",       "Supply",  "Supply voltage",             10,   24,   0.1,  21.6,   "V",    "U"),
  ("I",       "Motor",   "Current cap",                2,    16,   0.5,  12,     "A",    "I"),
  ("V",       "Motor",   "Max speed (12 V units)",     70,   720,  10,   700,    "",     "V"),
  ("d",       "Curve",   "Deadband",                   0.3,  3,    0.05, 1.0,    "kg",   "d"),
  ("Q_mm",    "Curve",   "Knee position",              0,    40,   0.5,  11,     "mm",   None),
  ("Q_kg",    "Curve",   "Knee force",                 1,    10,   0.05, 1.3,    "kg",   None),
  ("B_mm",    "Curve",   "Pad bite position",          0,    42,   0.5,  22,     "mm",   None),
  ("B_kg",    "Curve",   "Pad bite force",             1,    15,   0.1,  2.0,    "kg",   None),
  ("p",       "Curve",   "Full-travel force",          2,    40,   0.5,  20,     "kg",   "p"),
  ("L",       "Curve",   "Travel limit",               10,   48,   0.5,  44,     "mm",   "L"),
  ("k",       "Curve",   "Gamma (only with bite off)", 0.2,  3,    0.05, 0.6,    "",     "k"),
  ("y",       "Feel",    "Force hysteresis",           0,    2,    0.05, 0.25,   "kg",   "y"),
  ("w",       "Feel",    "Damping",                    0,    30,   0.5,  10,     "kg/100mm/s", "w"),
  ("W",       "Feel",    "Damping cap",                0,    10,   0.1,  1.5,    "kg",   "W"),
  ("a",       "ABS",     "ABS starts at",              2,    40,   0.25, 11,     "kg",   "a"),
  ("f",       "ABS",     "ABS full strength at",       2,    45,   0.25, 17.5,   "kg",   "f"),
  ("s",       "ABS",     "ABS pulse strength",         70,   720,  10,   600,    "",     "s"),
  ("h",       "ABS",     "ABS pulse rate",             3,    40,   1,    15,     "Hz",   "h"),
  ("bidir",   "ABS",     "Pulses back-and-forth",      0,    1,    1,    1,      "",     None),
  ("fx.abs_fade",  "Effects", "ABS collapse",          0,    1,    1,    0,      "",     None),
  ("fx.fade_s",    "Effects", "Collapse time",         0.2,  4,    0.1,  1.2,    "s",    None),
  ("fx.floor",     "Effects", "Collapse depth (lower=deeper)", 0.4, 1, 0.05, 0.85, "",   None),
  ("fx.back_s",    "Effects", "Firm-up time",          0.1,  3,    0.1,  0.5,    "s",    None),
  ("fx.hot",       "Effects", "Hot-brake fade",        0,    1,    1,    0,      "",     None),
  ("fx.heat_tau",  "Effects", "Cooling time",          5,    120,  1,    30,     "s",    None),
  ("fx.heat_on",   "Effects", "Fade starts at heat",   10,   400,  5,    60,     "",     None),
  ("fx.heat_full", "Effects", "Full fade at heat",     20,   600,  5,    140,    "",     None),
  ("fx.lock_kg",   "Effects", "Lock-up above",         3,    40,   0.5,  14,     "kg",   None),
  ("fx.soft",      "Effects", "Full fade softens by",  0,    0.9,  0.05, 0.5,    "",     None),
  ("fx.long_mm",   "Effects", "Full fade lengthens by", 0,   20,   0.5,  10,     "mm",   None),
  # game controller (USB HID gadget, /dev/hidg0): Pi side
  ("hid.brake_kg",    "Game", "Brake 100% at",         2,    60,   0.5,  20,     "kg",   None),
  ("hid.brake_dz",    "Game", "Brake deadzone",        0,    5,    0.05, 0.5,    "kg",   None),
  ("hid.brake_gamma", "Game", "Brake curve",           0.3,  3,    0.05, 1.0,    "",     None),
  ("hid.thr_max",     "Game", "Throttle 100% at",      50,   100,  0.5,  98,     "%",    None),
]
KN = {k[0]: dict(zip(("key","group","label","min","max","step","default","unit","cmd"), k)) for k in KNOBS}
DEFAULT_PROFILE = {k: v["default"] for k, v in KN.items()}

def fmt(v):
    return ("%.3f" % v).rstrip("0").rstrip(".")

def firmware_cmds(prof, p_live=None, b_mm_live=None):
    """Profile -> ordered firmware command list."""
    P = dict(DEFAULT_PROFILE, **prof)
    out = []
    for k in KNOBS:
        key, cmd = k[0], k[8]
        if key == "Q_mm":
            out.append("Q%s,%s" % (fmt(P["Q_mm"]), fmt(P["Q_kg"])))
        elif key == "B_mm":
            out.append("B%s,%s" % (fmt(b_mm_live if b_mm_live is not None else P["B_mm"]), fmt(P["B_kg"])))
        elif key == "p":
            out.append("p" + fmt(p_live if p_live is not None else P["p"]))
        elif cmd:
            v = P[key]
            out.append(cmd + (str(int(round(v))) if cmd in ("V", "s", "h") else fmt(v)))
    return out

# ---------------------------------------------------------------------------
class Hid:
    """USB HID game controller gadget (gadget/hid_gadget.sh makes /dev/hidg0).
    Report: buttons u8, X = brake u16, Y = throttle u16, both 0..4095.
    Non-blocking: if the PC is not reading, reports are dropped, never queued."""
    PATH = "/dev/hidg0"
    def __init__(self):
        self.fd = None; self.next_open = 0.0; self.status = "absent"; self.ok_t = 0.0
        self.last = None
    def send(self, brake, thr, force=False):
        """brake, thr in 0..1"""
        rep = struct.pack("<BHH", 0, int(round(max(0.0, min(1.0, brake)) * 4095)),
                          int(round(max(0.0, min(1.0, thr)) * 4095)))
        if self.fd is None:
            if time.time() < self.next_open: return
            try: self.fd = os.open(self.PATH, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:
                self.status = "absent"; self.next_open = time.time() + 2; return
        try:
            os.write(self.fd, rep); self.ok_t = time.time(); self.status = "ok"; self.last = rep
        except BlockingIOError:
            if time.time() - self.ok_t > 1: self.status = "no PC"
        except OSError:                      # gadget torn down / unbound
            try: os.close(self.fd)
            except OSError: pass
            self.fd = None; self.status = "absent"; self.next_open = time.time() + 2


class Pedal:
    """Serial owner + live state + effects."""
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.lock = threading.Lock()
        self.txq = queue.Queue()
        self.cond = threading.Condition()
        self.seq = 0
        self.live = {}                 # latest S-line fields
        self.log = []                  # recent non-S lines (seq, t, text)
        self.connected = False
        self.bidir_fw = True           # firmware state of the 'b' toggle (True after reset)
        self.stats = self._new_stats()
        self.rec = None                # open recording file
        self.state = self._load_state()
        self.profile = self._load_profile(self.state.get("profile", "default"))
        # effects state
        self.p_sent = None; self.b_sent = None
        self.abs_t = None; self.abs_floor = None; self.rel_t = None; self.rel_from = None
        self.heat = 0.0; self.last_t = None; self.fade = 0.0
        self.hid = Hid()

    # ---- persistence -------------------------------------------------------
    def _load_state(self):
        try: return json.load(open(STATE))
        except Exception: return {"profile": "default"}
    def _save_state(self):
        json.dump(self.state, open(STATE + ".tmp", "w"), indent=1); os.replace(STATE + ".tmp", STATE)
    def _load_profile(self, name):
        try: prof = json.load(open(os.path.join(PROFILES, name + ".json")))
        except Exception: prof = {}
        self.state["profile"] = name
        return dict(DEFAULT_PROFILE, **{k: v for k, v in prof.items() if k in KN})
    def profiles(self):
        os.makedirs(PROFILES, exist_ok=True)
        return sorted(f[:-5] for f in os.listdir(PROFILES) if f.endswith(".json"))
    def save_profile(self, name):
        if not re.fullmatch(r"[A-Za-z0-9 _.-]{1,40}", name): raise ValueError("bad name")
        os.makedirs(PROFILES, exist_ok=True)
        path = os.path.join(PROFILES, name + ".json")
        json.dump(self.profile, open(path + ".tmp", "w"), indent=1); os.replace(path + ".tmp", path)
        self.state["profile"] = name; self._save_state()
    def delete_profile(self, name):
        path = os.path.join(PROFILES, name + ".json")
        if os.path.exists(path): os.remove(path)
    def load_profile(self, name):
        self.profile = self._load_profile(name); self._save_state(); self.push_all()

    # ---- serial ------------------------------------------------------------
    def send(self, cmd):
        self.txq.put(cmd)
    def push_all(self):
        self.p_sent = self.b_sent = None
        for c in firmware_cmds(self.profile):
            self.send(c)
        self._sync_bidir()
    def _sync_bidir(self):
        want = bool(self.profile.get("bidir", 1))
        if want != self.bidir_fw:
            self.send("b")
    def set_knob(self, key, val):
        k = KN[key]
        val = max(k["min"], min(k["max"], float(val)))
        self.profile[key] = val
        if key in ("Q_mm", "Q_kg"):
            self.send("Q%s,%s" % (fmt(self.profile["Q_mm"]), fmt(self.profile["Q_kg"])))
        elif key in ("B_mm", "B_kg"):
            self.send("B%s,%s" % (fmt(self.profile["B_mm"]), fmt(self.profile["B_kg"]))); self.b_sent = None
        elif key == "bidir":
            self._sync_bidir()
        elif key == "U":
            self.send("U" + fmt(val)); self.send("V" + str(int(self.profile["V"])))
        elif key == "p":
            self.send("p" + fmt(val)); self.p_sent = None
        elif k["cmd"]:
            c = k["cmd"]
            self.send(c + (str(int(round(val))) if c in "Vsh" else fmt(val)))
        return val

    def writer(self):
        while True:
            cmd = self.txq.get()
            if cmd is None: return
            with self.lock:
                s = self.ser
            if not s: continue
            try:
                s.write((cmd + "\n").encode())
                self._note("> " + cmd, quiet=cmd[0] in "pB" and self._fx_on())
            except Exception:
                pass
            time.sleep(0.5 if cmd in ("g", "c", "t") else 0.03)

    def reader(self):
        while True:
            try:
                s = serial.Serial(self.port, 115200, timeout=0.5)
            except Exception:
                self._set_conn(False); time.sleep(2); continue
            with self.lock: self.ser = s
            self._set_conn(True)
            try:
                while True:
                    raw = s.readline()
                    if not raw: continue
                    self._line(raw.decode("ascii", "replace").strip())
            except Exception as e:
                self._note("serial lost: %s" % e)
            with self.lock: self.ser = None
            try: s.close()
            except Exception: pass
            self._set_conn(False); time.sleep(1)

    def _set_conn(self, c):
        if c != self.connected:
            self.connected = c
            if not c: self.hid.send(0, 0)    # never leave the game holding the brake
            self._note("Uno connected" if c else "Uno disconnected")
            self._bump()

    def _note(self, text, quiet=False):
        if quiet: return
        with self.cond:
            self.seq += 1
            self.log.append((self.seq, time.time(), text))
            del self.log[:-400]
            self.cond.notify_all()

    def _bump(self):
        with self.cond:
            self.seq += 1; self.cond.notify_all()

    def _new_stats(self):
        return {"peak_kg": 0.0, "zbig": 0, "zmax": 0, "resets": 0, "since": time.time()}

    def _line(self, ln):
        if self.rec:
            try: self.rec.write("%.3f %s\n" % (time.time(), ln))
            except Exception: pass
        if ln.startswith("S,"):
            p = ln.split(",")
            if len(p) < 13: return
            try:
                d = {"kg": float(p[1]), "cmd": int(p[2]), "mA": int(p[3]), "mode": p[4],
                     "mm": float(p[5]), "idx_bad": int(p[6]), "idx_n": int(p[7]), "z": p[8],
                     "lim11": p[9], "lim12": p[10], "kg_est": float(p[11]), "thr": float(p[12])}
            except ValueError:
                return
            d["t"] = time.time()
            self.stats["peak_kg"] = max(self.stats["peak_kg"], d["kg"])
            try: self.stats["zmax"] = max(self.stats["zmax"], int(d["z"].split("/")[1]))
            except Exception: pass
            self._effects(d)
            self._game(d)
            with self.cond:
                self.live = d; self.seq += 1; self.cond.notify_all()
            return
        if ln.startswith("ZBIG"):
            self.stats["zbig"] += 1
        if "INTERACTIVE SIM" in ln:          # boot banner: the Uno (re)started
            self.stats["resets"] += 1
            self.bidir_fw = True
            self._note(ln)
            self._note("Uno started - pushing profile '%s'" % self.state.get("profile"))
            self.push_all()
            return
        if ln.startswith("ABS bidirectional="):
            self.bidir_fw = ln.endswith("1")
        if ln.startswith(("full travel at", "pad bite")) and self._fx_on():
            return                           # effect updates: don't flood the log
        if ln.startswith("ZBIG"):
            return
        self._note(ln)

    # ---- game controller ---------------------------------------------------
    def _game(self, d):
        P = self.profile
        dz = P["hid.brake_dz"]
        b = max(0.0, d["kg"] - dz) / max(0.1, P["hid.brake_kg"] - dz)
        b = min(1.0, b) ** P["hid.brake_gamma"]
        t = min(1.0, d["thr"] / P["hid.thr_max"])
        if d["mode"].startswith("cal"): b = 0.0   # calibration pushes the pedal, not the driver
        self.hid.send(b, t)
        d["g_brk"] = round(b * 100, 1); d["g_thr"] = round(t * 100, 1)

    # ---- effects (Pi side) -------------------------------------------------
    def _fx_on(self):
        P = self.profile
        return bool(P.get("fx.abs_fade")) or bool(P.get("fx.hot"))

    def _effects(self, d):
        P = self.profile
        now = d["t"]
        dt = 0.05 if self.last_t is None else min(0.2, now - self.last_t); self.last_t = now
        p0, bmm, bkg = P["p"], P["B_mm"], P["B_kg"]
        p_min = bkg + 0.7                     # firmware drops the bite curve below bite_kg+0.5
        p_want, b_want = p0, bmm
        kg = d["kg"]
        # hot brakes: heat builds with braking, unless locked; always cools
        if P.get("fx.hot"):
            if bkg < kg < P["fx.lock_kg"]: self.heat += kg * dt
            self.heat -= self.heat * dt / max(1.0, P["fx.heat_tau"])
            span = max(1.0, P["fx.heat_full"] - P["fx.heat_on"])
            self.fade = min(1.0, max(0.0, (self.heat - P["fx.heat_on"]) / span))
            p_want = max(p_min, p0 * (1 - P["fx.soft"] * self.fade))
            b_want = bmm + P["fx.long_mm"] * self.fade
        else:
            self.heat = 0.0; self.fade = 0.0
        # ABS collapse: p ramps down while ABS runs, back up after
        if P.get("fx.abs_fade"):
            if d["mode"].startswith("ABS"):
                if self.abs_t is None:
                    self.abs_t, self.rel_t = now, None
                    self.abs_floor = max(p_min, P["fx.floor"] * kg)
                u = min(1.0, (now - self.abs_t) / max(0.05, P["fx.fade_s"]))
                p_want = min(p_want, p_want - (p_want - self.abs_floor) * u)
            else:
                if self.abs_t is not None:
                    self.abs_t, self.rel_t, self.rel_from = None, now, (self.p_sent or p_want)
                if self.rel_t is not None:
                    u = min(1.0, (now - self.rel_t) / max(0.05, P["fx.back_s"]))
                    p_want = self.rel_from + (p_want - self.rel_from) * u
                    if u >= 1: self.rel_t = None
        else:
            self.abs_t = self.rel_t = None
        p_want = max(p_min, p_want)
        if self.p_sent is None or abs(p_want - self.p_sent) >= 0.2 or (p_want == p0 and self.p_sent != p0):
            if self.txq.qsize() < 4:
                self.send("p%.2f" % p_want); self.p_sent = p_want
        if self.b_sent is None or abs(b_want - self.b_sent) >= 0.3 or (b_want == bmm and self.b_sent != bmm):
            if self.txq.qsize() < 4:
                self.send("B%.1f,%s" % (b_want, fmt(bkg))); self.b_sent = b_want
        d["heat"] = round(self.heat, 1); d["fade"] = round(self.fade, 3); d["p_live"] = round(p_want, 2)
        d["b_live"] = round(b_want, 1)

    # ---- recording ---------------------------------------------------------
    def record(self, on):
        if on and not self.rec:
            os.makedirs(LOGS, exist_ok=True)
            name = time.strftime("rec-%Y%m%d-%H%M%S.txt")
            self.rec = open(os.path.join(LOGS, name), "w")
            self.rec.write("# profile %s %s\n" % (self.state.get("profile"), json.dumps(self.profile)))
            self._note("recording -> logs/" + name)
        elif not on and self.rec:
            self.rec.close(); self.rec = None; self._note("recording stopped")

    def snapshot(self, since_log=0):
        return {"seq": self.seq, "connected": self.connected, "live": self.live,
                "log": [l for l in self.log if l[0] > since_log],
                "stats": self.stats, "recording": bool(self.rec), "hid": self.hid.status,
                "profile_name": self.state.get("profile")}


# ---------------------------------------------------------------------------
PED = None
INDEX = os.path.join(HERE, "index.html")

class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, *a): pass

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            b = open(INDEX, "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(b); return
        if self.path == "/api/config":
            return self._json({"knobs": [KN[k[0]] for k in KNOBS], "profile": PED.profile,
                               "profiles": PED.profiles(), "profile_name": PED.state.get("profile")})
        if self.path == "/api/stream":
            return self._stream()
        self._json({"error": "not found"}, 404)

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last_log = PED.log[-60][0] - 1 if len(PED.log) >= 60 else 0
        seq = -1
        try:
            while True:
                with PED.cond:
                    PED.cond.wait_for(lambda: PED.seq != seq, timeout=2.0)
                    snap = PED.snapshot(last_log); seq = snap["seq"]
                if snap["log"]: last_log = snap["log"][-1][0]
                self.wfile.write(("data: %s\n\n" % json.dumps(snap)).encode())
                self.wfile.flush()
                time.sleep(0.04)              # <= ~25 updates/s per browser
        except Exception:
            pass
        self.close_connection = True

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try: body = json.loads(self.rfile.read(n) or b"{}")
        except Exception: body = {}
        try:
            r = self.route(body)
        except Exception as e:
            return self._json({"error": str(e)}, 400)
        self._json(r or {"ok": True})

    def route(self, b):
        p = self.path
        if p == "/api/knob":
            return {"value": PED.set_knob(b["key"], b["value"])}
        if p == "/api/arm":      PED.send("g"); return
        if p == "/api/disarm":   PED.send("x"); return
        if p == "/api/calibrate": PED.send("c"); return
        if p == "/api/tare":     PED.send("t"); return
        if p == "/api/push":     PED.push_all(); return
        if p == "/api/cmd":
            c = str(b.get("cmd", "")).strip()
            if not c or len(c) > 30 or c[0] in "Mv": raise ValueError("refused (test commands M/v block the pedal)")
            PED.send(c); return
        if p == "/api/profile/load":   PED.load_profile(b["name"]); return {"profile": PED.profile}
        if p == "/api/profile/save":   PED.save_profile(b["name"]); return {"profiles": PED.profiles()}
        if p == "/api/profile/delete": PED.delete_profile(b["name"]); return {"profiles": PED.profiles()}
        if p == "/api/stats/reset":    PED.stats = PED._new_stats(); return
        if p == "/api/record":         PED.record(bool(b.get("on"))); return
        raise ValueError("unknown endpoint")


def main():
    global PED
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--http", type=int, default=8080)
    a = ap.parse_args()
    PED = Pedal(a.port)
    if not os.path.exists(os.path.join(PROFILES, "default.json")):
        PED.save_profile("default")
    threading.Thread(target=PED.writer, daemon=True).start()
    threading.Thread(target=PED.reader, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.http), H)
    srv.daemon_threads = True

    def bye(*_):
        try:
            with PED.lock:
                if PED.ser: PED.ser.write(b"x\n"); PED.ser.flush()
        except Exception: pass
        PED.record(False)
        os._exit(0)
    signal.signal(signal.SIGTERM, bye); signal.signal(signal.SIGINT, bye)
    print("pedald: serial %s, web http://0.0.0.0:%d" % (a.port, a.http), flush=True)
    srv.serve_forever()

if __name__ == "__main__":
    main()
