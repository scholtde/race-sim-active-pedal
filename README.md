# race-sim-active-pedal

A DIY **active brake pedal** for sim racing. A load cell measures how hard you
push, and a DC motor on a lead screw moves the pedal to wherever that force
*should* put it. The motor is driven by an Arduino and tuned live from a web
page on a Raspberry Pi.

Because the pedal is driven rather than sprung, its feel is software:

- the free travel, the point where the pads bite, and the stiffness after the bite
- damping
- ABS pulsing
- an ABS "collapse" under the foot
- brake fade from heat

Every one of these is a setting you can change while you drive.

> Status: working prototype on the bench / floor board. Brake + throttle are
> read and tuned, and reach the game PC as a USB game controller ("Active Pedal").

---

## How it works

```
             force (kg)                     position (mm)
 foot ──► load cell ──► HX711 ──► Uno ──► VNH5019 ──► DC motor ──► TR12x3 screw ──► pedal
                                   ▲                                    │
                                   └──────── encoder (2048 counts/rev) ◄┘
                                   │
                 USB serial (115200)
                                   │
                         UP board: pedald.py ── network ──► web GUI (any browser)
                                              └ USB OTG (HID joystick) ──► game PC
```

- **Arduino Uno (`firmware/pedal_sim`)** runs the real-time loop:
  - turns force into a target position through the pedal curve
  - holds that position with a position loop
  - adds damping, ABS pulses and the safety layers:
    - limit switches
    - soft limits
    - current cap
    - load-cell watchdog
    - encoder index (Z) correction
    - force ceiling
- **Raspberry Pi (`web/pedald.py`)** owns the serial port and keeps the
  settings:
  - It pushes the active profile to the Uno on every Uno start, so a power blip
    never loses your tune.
  - It runs Pi-side effects that the Uno has no flash left for (ABS collapse,
    hot-brake fade).
  - It serves the web GUI.

The Uno's flash is full (32,242 of 32,256 bytes). New features go Pi-side, or
onto a bigger MCU.

### The pedal curve

```
force
  │                                    ╱  p   (full-travel force, at L)
  │                                 ╱
  │                              ╱     stiff: pads on the rotor
  │                 B ●──────╯ bite (B_mm, B_kg)
  │            ╱────╯          firmer: taking up the slack
  │  Q ●──────╯ knee (Q_mm, Q_kg)
  │ ╱──╯                       near-free first part of the travel
  └──────────────────────────────────── travel (mm)
```

ABS is triggered by **force**: it starts at `a` kg and reaches full strength at
`f` kg.

---

## Hardware

| Part | Notes |
|---|---|
| Arduino Uno R3 | runs `pedal_sim`; USB to the Pi |
| Pololu Dual VNH5019 motor shield | motor on **M1**; M2 jumpers removed; **remove the VIN jumper** above 20 V |
| DC motor | Bühler transfer-case actuator (Jeep WK2, 1.13.064.407), measured R 0.386 Ω, Ke 0.120 V per count/ms |
| TR12x3 trapezoidal lead screw | motor and encoder directly on opposite ends; 682.7 counts/mm |
| Encoder E6B2-CWZ3E | 1024 ppr (×2 decode = 2048 counts/rev), **5 V** supply |
| Load cell YZC-516C, 200 kg (2 mV/V) + HX711 | between the rail and the pedal arm |
| 2 limit switches | NC to GND, pull-ups; define home and end of travel |
| Throttle | potentiometer on a pedal, A2 |
| Supply | 12 V works. The rig is tuned at **21–21.6 V** (24 V PSU trimmed down). Fit 4,700–10,000 µF / 35 V at the shield; a 26–28 V TVS is cheap insurance. Bench PSUs cannot absorb regen. |
| Host | UP board UP-CHT01 (Atom x5-Z8350): runs pedald and is the USB game controller via its OTG port (`gadget/`). A Pi 3B+ works for pedald but cannot be a USB device. |

### Uno pins

| Pin | Use |
|---|---|
| D3 | encoder A (INT1) |
| D8 | encoder B |
| D10 | encoder Z (index) |
| D5 / D7 | HX711 SCK / DOUT |
| D11 | limit switch, motor end (pedal fully pressed) |
| D12 | limit switch, encoder end (home, pedal released) |
| A2 | throttle potentiometer |
| D2, D4, D6, D9, A0 | VNH5019 M1 (INA, INB, EN, PWM, CS), used by the shield |

> `md.init()` from the Pololu library claims the M2 pins (D7/D8/D10) and turns
> on Timer1 PWM on D10. The firmware takes them back right after; any new
> sketch for this wiring must do the same.

---

## Repository layout

```
firmware/
  pedal_sim/        the pedal firmware (pedal_sim.ino + calibrate / motortest / speedtest tabs)
  pedal_probe/      read-only probe: encoder, switches, load cell, throttle (no motor)
  bringup/          the bring-up sketches used to characterise each part
web/
  pedald.py         Pi daemon: serial owner, profiles, effects, web server (stdlib + pyserial)
  index.html        the web GUI (single file, no internet needed)
  profiles/         saved profiles (JSON); default.json = the current 21 V tune
tools/              command-line helpers (run these when pedald is NOT running)
  monitor.py        calibrate + arm + live brake/throttle bars; monitor.py <secs> [cmds...]
  fade_lab.py       the ABS-collapse / hot-fade experiment that became the web effects
  cal_run.py        run one calibration and print it
  probe_log.py      log pedal_probe output
  mt_run.py         motor characterisation ('M')
  loop.py, tune.py  offline position-loop simulation / tuning
```

---

## Getting started (Raspberry Pi)

```bash
sudo apt install python3-serial git
git clone https://github.com/scholtde/race-sim-active-pedal.git ~/projects/race-sim-active-pedal
cd ~/projects/race-sim-active-pedal
```

### Build and flash the firmware

With [arduino-cli](https://arduino.github.io/arduino-cli/), the AVR core and the
*DualVNH5019MotorShield* library installed:

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/pedal_sim
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyACM0 firmware/pedal_sim
```

### Run the web GUI

```bash
python3 web/pedald.py            # --port /dev/ttyACM0 --http 8080
```

Open **http://&lt;pi-address&gt;:8080**.

- **Arm** calibrates first (the pedal drives to both switches; keep your foot
  off), then arms.
- Every slider applies live. **Save** writes the profile.
- **Record** logs raw telemetry to `web/logs/`.
- Stopping `pedald` (Ctrl-C / SIGTERM) disarms the pedal.

### Start on boot (systemd)

```bash
sudo cp web/pedald.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pedald
```

| Task | Command |
|---|---|
| Status | `systemctl status pedald` |
| Follow the log | `journalctl -u pedald -f` |
| Restart after a code change | `sudo systemctl restart pedald` |

The unit assumes user `simpedal`, the repo at `~/projects/race-sim-active-pedal`
and a Python venv at `<repo>/venv`. Edit it if yours differ. On an OS with an
old Python (the UP board runs Ubuntu 16.04 / Python 3.5; pedald needs 3.7+):

```bash
uv venv --python 3.12 venv && uv pip install --python venv/bin/python pyserial
```

Only one program may own the serial port. Stop `pedald` before using the
`tools/` scripts (`sudo systemctl stop pedald`), and start it again afterwards.

### Game controller (USB HID)

The host's OTG port shows up on the PC as **Active Pedal**: X = brake (from the
load-cell force), Y = throttle, 0–4095 each, updated with every telemetry line
(20 Hz). The *Game* settings set the force for 100% brake, a deadzone and a curve.
It needs a board with a USB device controller (`ls /sys/class/udc`): UP-CHT01,
Pi Zero / 4 / 5. A Pi 3B+ cannot do it.

```bash
sudo cp gadget/99-pedal-hidg.rules /etc/udev/rules.d/
sudo cp gadget/pedal-gadget.service /etc/systemd/system/ && sudo systemctl daemon-reload
sudo systemctl enable --now pedal-gadget && sudo systemctl restart pedald
```

On the UP board the firmware forces the OTG port to host mode, and its 4.15
kernel has no driver to change that; `gadget/otg_role.py device` sets the
switch directly (the service does it at every boot). Test without pedald:
`gadget/sweep.py`, then watch `joy.cpl` on Windows. If Windows shows the device
but not the controller after a descriptor change, bump `bcdDevice` in
`hid_gadget.sh` (Windows caches descriptors).

### Command-line alternative

```bash
cd tools
python3 -u monitor.py 120 U21.6 V700 I12 Q11,1.3 B22,2 p20 a11 f17.5 s600 h15 L44 w10 W1.5
```

---

## Settings

All of these are in the web GUI. The serial command is shown for the CLI.

| Setting | Cmd | Meaning |
|---|---|---|
| Supply voltage | `U<V>` | the real supply voltage, 10–24 V. Motor maths stays in 12 V units; this only scales the output (and raises the speed cap to 400×U/12). **Never set it lower than the real supply.** |
| Current cap | `I<A>` | motor current limit (model-based) |
| Max speed | `V<n>` | speed cap, in 12 V units |
| Deadband | `d<kg>` | below this the motor does nothing |
| Knee | `Q<mm>,<kg>` | end of the near-free travel (`Q0` = off) |
| Pad bite | `B<mm>,<kg>` | where the pads touch (`B0` = off, then gamma `k` shapes the curve) |
| Full-travel force | `p<kg>` | force for full travel |
| Travel limit | `L<mm>` | usable travel |
| Hysteresis | `y<kg>` | force hysteresis against hunting |
| Damping / cap | `w<n>` / `W<kg>` | virtual damping (kg per 100 mm/s) and its cap; w10 W1.5 stops foot-spring hunting |
| ABS start / full | `a<kg>` / `f<kg>` | ABS onset and full-strength force |
| ABS strength / rate | `s<n>` / `h<Hz>` | pulse amplitude and frequency |
| Pulse direction | `b` (toggle) | back-and-forth or one-way knocks |
| Effects | Pi only | ABS collapse (fade time, depth, firm-up), hot-brake fade (heat build / cooling, lock-up, softening, lengthening) |

Other commands: `g` arm, `x` disarm, `c` calibrate, `t` tare, `?` help.

The telemetry line is `S,…` every 50 ms with these fields: force, speed
command, motor mA, mode, mm, bad index checks, index count, last/max Z
correction, both switches, force estimate, throttle %.

---

## Safety

This is a motor that pushes on your foot.

- It starts **disarmed** with the bridge off (the shaft turns freely).
- There is no motion until a calibration has measured the travel.
- Soft limits sit 1.5 mm inside each switch, with speed tapering near the ends.
- A switch trip brakes the motor in the interrupt and disarms.
- The motor stops when force drops below the deadband, so letting go stops it.
- There is a current cap, a hard force ceiling, a load-cell watchdog, and ABS
  bursts are capped at 5 s.
- Encoder Z corrections over 256 counts are treated as a lost position.

Keep the encoder cable away from the motor leads; use shielded cable with the
shield grounded at the Arduino end. Electrical noise there shows up as Z
corrections in the GUI's *Encoder health* panel.

---

## Roadmap

- [x] Game link: USB HID game controller (brake + throttle) from the UP board
- [ ] Faster telemetry for the game axes (20 Hz now; the load cell runs at ~86 Hz)
- [x] `pedald` as a systemd service
- [x] One host board (UP-CHT01) for pedald, GUI and game link; the 3B+ is retired
- [ ] Tactile transducer / vibration motor for ABS kicks
- [ ] Game telemetry driven effects (ABS, lock-up, fade from the sim)
- [ ] Shielded encoder cable; bigger MCU when the Uno runs out

## License

GPL-3.0, see [LICENSE](LICENSE).
