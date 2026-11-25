/*
 * Active pedal -- INTERACTIVE SIM, coupled to the lead screw.
 *
 * Press the load cell -> the motor drives the rail toward the motor end at a
 * speed proportional to force. Push past the ABS threshold -> the motor pulses
 * back and forth like ABS modulation. Release -> the rail returns to park.
 *
 * Mechanics: motor and encoder directly on opposite ends of the TR12x3 screw,
 * 2048 counts/rev -> 682.7 counts/mm. Travel between the limit switches is
 * MEASURED by the calibration (calibrate.ino) every time it is armed from
 * cold: the switches get re-adjusted as the project goes.
 *
 * Safety, in order of how likely it is to save something:
 *   - starts DISARMED with the bridge disabled (D6 low): shaft free by hand
 *   - 'g' calibrates first if needed; no motion without a measured travel
 *   - soft limits 1.5 mm inside each switch, with speed tapering to the
 *     minimum over the last 5 mm, so normal use never reaches a switch
 *   - limit switch toward the moving direction BRAKES in the ISR. That means
 *     the soft limit was missed: disarm and demand a recalibration
 *   - motor stops the instant force drops below the deadband (let go = stop)
 *   - hard force ceiling, hard current limit, load-cell watchdog
 *   - slew-rate limited in follow mode so speed changes can't cause inrush
 *   - ABS bursts are time-capped; this motor is an intermittent-duty actuator
 *
 * HX711 DOUT=D7 SCK=D5 (~86 SPS).  Motor on M1 (M2 unused).  Serial 115200.
 * Encoder A=D3 (INT1, x2) B=D8 Z=D10.  Limits NC to GND with pull-ups:
 * D11 = motor end (CW, counts up, pedal pressed), D12 = encoder end (home).
 * Commands:  g = arm   x = disarm/stop   c = calibrate   t = tare   ? = help
 */
#include <DualVNH5019MotorShield.h>
#include "cal_state.h"   // the enum must precede Arduino's generated prototypes
DualVNH5019MotorShield md;

const uint8_t HX_DOUT = 7, HX_SCK = 5;   // DOUT moved off D3 (INT1) for the encoder

// ---- calibration -----------------------------------------------------------
// 200 kg cell, 2 mV/V (fitted 2026-11-02). Scaled from the old DB-10 fit
// (45,975 counts/kg at ~2.17 mV/V, 100 kg): 45975 * 2.0/2.17 * 100/200.
// NEGATIVE: a press reads negative raw on this cell. Not yet checked against
// a known weight -- the cell's own tolerance is typically a few %.
const float COUNTS_PER_KG = -21187.0;
long  force_zero = 0;                  // tared on arm

// ---- feel / mapping --------------------------------------------------------
float force_deadband =  1.0;   // kg below which the motor stays still (cmd: d<kg>)
float abs_threshold  = 15.0;   // kg at which ABS modulation kicks in  (cmd: a<kg>)
const float FORCE_MAX      = 60.0;   // kg hard cutoff
const int   SPEED_MIN      =  70;    // motor won't turn much below this
// If the position were ever wrong the rail could reach a switch at SPEED_MAX,
// and the levers have only 1.38 mm (home) / 1.48 mm (end) past the trip.
// Braking distance measured mid-travel with 'v' (2026-10-17), ~linear in speed:
//    speed 100 ~27 mm/s -> 0.45-0.51 mm
//    speed 150  43 mm/s -> 0.83-0.86 mm   (+0.3 mm envelope = 1.16: fits)
//    speed 200  61 mm/s -> 1.22-1.26 mm   (+0.3 mm envelope = 1.56: does NOT)
//    speed 300  ~90 mm/s -> ~1.9 mm, 400 125 mm/s -> ~2.7 mm  (with miscounts)
// x2 decoding miscounts from ~85 mm/s (B read late behind other ISRs). The Z
// index puts those counts back once per rev (see the PCINT ISR), so the cap is
// a runtime setting now (cmd: V<speed>, 70..400).
//
// Stopping distance is ~linear in speed: ~4.5 counts per speed unit (0.0066 mm).
// Every move is capped so that TWICE that fits in the room left: the motor
// also needs time to slow when the PWM drops. That one rule replaces the old
// fixed 5 mm taper, so a 400 stomp starts slowing ~5 mm out, a 150 one ~2 mm.
int         speed_max      = 200;
const int   SPEED_LIMIT    = 400;   // at 12 V; U<volts> raises it (400 x V/12)
// Supply voltage (cmd: U<volts>, 10-24). ALL motor maths (model, current cap,
// speeds, ABS amplitude) is in 12 V units; pwm12() converts at the output, so
// a higher supply changes nothing but the top speed you are allowed to ask for.
float v_scale = 1.0;                   // 12 / supply volts
int pwm12(int o) { return constrain((int)(o * v_scale), -400, 400); }
const long  STOP_K         = 9;      // counts of room per speed unit (2 x 4.5)
// Motor model, MEASURED with 'M' (motortest.ino, 2026-10-17, 12 V supply):
//   V*duty = Ke*v + R*I + Vd,  Ke = 0.120 V per count/ms, R = 0.386 ohm
//   (winding + driver + wiring), Vd = 0.49 V; fits 15 points within 0.1 V.
//   Steady speed v = 0.233*cmd - 4.7 counts/ms (400 -> 87 counts/ms).
//   CS reads MOTOR current (linear in duty at stall, not duty^2); the VNH5019
//   PWMs the low side, so supply current = duty x CS.
const float V_PER_UNIT     = 0.233;  // counts/ms per speed unit
// Current cap: cmd <= (400/V) * (i_cap_a*R + Vd + Ke*v) = 12.9*A + 16.3 + 4.0*v.
// The first version assumed R = 0.75 ohm and so allowed 18 A at standstill --
// the 16-18 A peaks seen in stomps. Set to the PSU's limit (cmd: I<amps>).
float       i_cap_a        = 10.0;
const float I_CAP_PER_A    = 12.87;  // command per amp (0.386 ohm x 33.3/V)
const float I_CAP_OFFSET   = 16.3;   // the 0.49 V drop, in command units
const float I_CAP_PER_V    = 3.99;   // command per count/ms of shaft speed (Ke x 33.3)
int         abs_speed      = 130;    // amplitude at abs_full_kg (cmd: s<n>)
float       abs_full_kg    = 25.0;   // force at which ABS reaches full amplitude (f<kg>)
const int   ABS_SPEED_MIN  =  70;    // amplitude at the trigger; below ~70 the
                                     // motor doesn't reliably turn at all
bool        abs_bidir      = true;   // false = pulse one way (drive/coast):
                                     // no reversal, so no back-EMF current spike.
                                     // true needs a supply that can take the surge.
// Drive/coast split per half-cycle; full period = 2*(drive+coast). Tunable live
// with h<hz>. Measured reversal current is 30.4*(speed/400) A, so at speed 130
// that is ~9.9 A -- right on the supply's 10 A ceiling.
unsigned long abs_drive_ms = 18;          // drive each direction...
unsigned long abs_coast_ms =  7;          // ...with a coast between  -> ~20 Hz
const unsigned long ABS_MAX_MS   = 5000;  // max continuous ABS burst (thermal):
                                          // bidirectional runs near stall current
const float SLEW_PER_MS    = 2.0;    // follow-mode speed slew (full scale/200ms)

// ---- travel ----------------------------------------------------------------
const long  CPR            = 2048;
const float COUNTS_PER_MM  = CPR / 3.0;  // TR12x3, direct drive
const long  SOFT_MARGIN    = 1024;   // 1.5 mm inside each switch trip point;
                                     // the switch re-closes ~500 counts inside
const long  TAPER_COUNTS   = 3413;   // 5 mm: minimum useful travel; the return's slow-down zone
const long  PARK_COUNTS    = 1365;   // 2 mm off the home trip: the rest position
const long  PARK_EPS       =  100;
const int   THR_MIN = 278, THR_MAX = 645; // throttle pot A2 -> 0..100 %; rest 266-272, WOT 646-658 (2026-11-24, pot moved in the floor-board move)
// Encoder envelope: a backup for a switch that fails to trip (e.g. a bent
// lever). Past either trip point by this much -> brake and disarm, whatever
// the guard says. Near the ends the stop cap keeps the speed low enough that
// braking adds well under 1 mm.
const long  ENVELOPE_COUNTS = 205;   // 0.3 mm
// Encoder-motion watchdog: driving at >= WD_MIN_SPEED for WD_WINDOW_MS must
// move the encoder >= WD_MIN_COUNTS.
// Less = slipped coupling, dead encoder or a jam; position is then frozen and
// the soft limits are blind.
// 10 ms window: at 400 (125 mm/s) a dead encoder costs ~1.3 mm before the
// brake, not ~3 mm. Speed 120 moves ~250 counts in 10 ms.
const int   WD_MIN_SPEED   = 120;
const unsigned long WD_WINDOW_MS = 10;
const long  WD_MIN_COUNTS  = 20;
// Only while actually DRIVING, and not in the first 15 ms of a drive: braking a
// shaft that is still running the other way is not an encoder fault (it was
// once: a re-press during a 400 return tripped it at -593 counts).
const unsigned long WD_GRACE_MS = 15;
const int   WD_FREE_MA     = 4500;   // below this a motor that isn't turning the encoder is spinning free
const int   RETURN_SPEED   = 150;   // tapers to SPEED_MIN near park

// ---- pedal (position mode) -------------------------------------------------
// Force sets a target POSITION through a progressive curve -- travel comes
// easily at first and stiffens toward the bottom, like a real brake pedal --
// and a position loop drives there. 'm' toggles back to speed-follow.
bool  pos_mode       = true;
// Hand range measured 2026-10-17 (cell not yet in the pedal): firm presses
// 21-25 kg, hardest ~30 kg. Full travel at a firm press; ABS in the top part.
float pedal_full_kg  = 25.0;  // force for full travel            (cmd: p<kg>)
float pedal_gamma    = 0.6;   // x = F^gamma: <1 = progressive    (cmd: k<g>)
const long CURVE_TOP_MARGIN     = 341;   // 0.5 mm: full force stops short of the soft limit
// Optional cap on full pedal travel (cmd: L<mm>, L0 = off). The screw is bent
// over its last ~20 mm and worst at ~47 mm: L44 keeps pressing and ABS out of
// the heaviest part.
long  travel_limit   = 0;               // counts; 0 = use the whole soft range
// Pad bite (cmd: B<mm>,<kg>; B0 = off -> plain gamma curve). Light travel from
// park to bite_mm (deadband..bite_kg), then the pads "touch": force rises
// steeply and keeps stiffening, F = bite + (full-bite)*(a*t + (1-a)*t^2).
float bite_mm = 10.0, bite_kg = 2.5;
// Optional knee in the free travel (cmd: Q<mm>,<kg>; Q0 = off): park..q_mm over
// deadband..q_kg (near-free), then q_mm..bite_mm over q_kg..bite_kg. Needs B on.
float q_mm = 0, q_kg = 1.2;
// 1.0 = constant stiffness after contact: a clear step (~2.7x at the defaults).
// 0.5 started the pads SOFTER than average -- barely stiffer than the free
// travel, so the bite hardly showed and the rail hunted just past it.
const float BITE_A = 1.0;
// Pedal-arm drag. The cell sits between the rail and a pedal arm with
// friction, so while the rail MOVES the cell reads the arm's drag on top of the
// foot: -3.9 kg pressing, ~+0.5 kg returning (2026-11-03, no hand). The
// position loop then saw the force collapse every time it moved deeper, and
// hunted. Calibration measures it; the pedal adds DRAG_GAIN of it back while
// moving (scaled in over DRAG_V0 counts/ms). <1 so it can't over-correct.
// TURNED OFF (gain 0) after one run: cancelling the friction removed the loop's
// damping and the rail hunted 4-5 mm at ~3 Hz against the hand (a loop sim
// reproduced it: 6 mm). Still measured and printed on arm, for reference.
const float DRAG_GAIN = 0.0, DRAG_V0 = 10.0;
// Virtual damping (cmd: w<kg per 100 mm/s>). The hand is a spring (~1 kg/mm
// seen on the bench): the rail yields, the force drops, the target backs off --
// with ~70 ms of force-filter delay that loop hunts. Reading the force as
// lower while the pedal moves deeper (higher while it comes back) damps it,
// like a brake's hydraulics. Simulated over 1.8-18 kg and hand stiffness
// 0.5-3 kg/mm: worst hunt 6 mm -> ~0.3 mm. A target low-pass made it WORSE.
// Only while the hand is on (raw force >= deadband): a release must not be
// slowed by a phantom force.
// 0.025 still left a +-1 mm, 5 Hz hunt just past the bite on the bench.
float pedal_damp = 0.1465;              // kg per count/ms (= 10 kg per 100 mm/s)
// 2026-11-24, foot on a rolling chair: the leg hunts ~5 Hz, +-1 mm (~30 mm/s).
// w10 calmed it but capped stabs at ~43 mm/s (10 kg of drag at 100 mm/s, more
// than the 7 kg foot). So the damping FORCE is capped (cmd W<kg>, 0 = no cap):
// full damping on the small, slow hunting motion, a fixed small drag on stabs.
float damp_cap = 1.5;                  // 2026-11-24 foot test: 1.5 = 0% hunting + 120 mm/s stabs; 3, 5 and uncapped all hunted 10-16%
float drag_fwd_kg = 0, drag_bwd_kg = 0;
long  drag_raw_f = 0, drag_raw_b = 0; bool drag_ok = false;
// Force hysteresis (cmd: y<kg>): the target only follows once the force has
// moved this far, like the friction in a real pedal. Stops small wobbles
// turning into motion.
float force_hyst = 0.25;
float force_est = 0, force_b = 0;       // drag-compensated / after hysteresis
bool  hx_new = false;
const long POS_HOLD_ENTER = 68;          // 0.1 mm: brake and hold inside this...
const long POS_HOLD_EXIT  = 205;         // ...until the error grows past 0.3 mm

// ---- limits ----------------------------------------------------------------
// Measured 15.9 A on a 20 Hz reversal at speed 130 -- higher ABS frequency means
// less coast time, so less back-EMF decay, so more current. Set below the motor's
// ~16 A stall and inside the VNH5019's 30 A peak, and let the 10 A supply be the
// real limiter: clamping lower just chopped the pulse into constant backoff.
// 14 A made ABS back off half the time in the bent part of the screw (ABS-oc,
// 2026-11-03), which is also why it felt weak; the VNH5019 takes 30 A peaks.
const int MAX_CURRENT_MA = 17000;
const unsigned long HX_WATCHDOG_MS = 200;   // 80 SPS = a sample every 11.5 ms
const unsigned long OC_BACKOFF_MS  =  60;   // motor rest after a current spike
const uint8_t       OC_MAX_STRIKES =  40;   // ...before we conclude it's stuck

// ---- encoder + limit switches (ISR state) ------------------------------------
volatile long enc = 0;
volatile int8_t  guard = 0;            // +1 moving CW: D11 brakes; -1 CCW: D12 brakes
// Pedal mode also brakes on a switch when the SHAFT is moving toward it (a_dir),
// whatever was last commanded: once, the command went negative near the top
// while the rail was still coasting forward at 400, so the guard watched D12
// and the rail ran through the D11 trip. Off in calibration, which backs away
// from an already-open switch.
volatile bool    motion_guard = false;
volatile int8_t  a_dir = 0;            // direction of the last counted A edge
volatile bool    z_fast = false;       // shaft fast enough that a Z pulse can be missed
volatile bool    braked = false;
volatile long    hit_cnt = 0;
volatile uint8_t hit_pin = 0;          // 11 or 12
volatile uint8_t pb_prev = 0;
volatile long idx_last = 0;
volatile bool idx_seen = false;
volatile unsigned long idx_n = 0, idx_bad = 0;
// Z correction. Z fires at one physical point per rev, so its count modulo
// 2048 is fixed -- one phase per direction, since the rising edge is the
// leading edge of the Z pulse whichever way the shaft turns. Calibration
// learns both at speed 100 (clean); after that every Z snaps the count back
// onto the right phase, putting back whatever x2 decoding lost that rev.
volatile bool    z_learn = false, z_ok = false, z_fault = false;
volatile int16_t z_phase[2];           // [0] = CW (counts up), [1] = CCW
volatile uint8_t zl_n[2];
volatile uint8_t zl_bad = 0;
volatile unsigned long z_corr_n = 0;   // corrections applied (non-zero)
volatile int16_t z_corr_max = 0, z_fault_d = 0;
// 64 was hit once (70 counts) mid-stomp at 400 -- either Z pulses missed (a
// few us wide at full speed: the ISR can run after Z has dropped again) or
// brake-current noise. 256 (0.37 mm) is still a quarter of the 1024-count
// ambiguity and well inside the 1.5 mm soft margin.
const int16_t Z_MAX_CORR = 256;        // more than this in one go = lost, not miscounted
const int16_t Z_REPORT   = 24;         // report corrections bigger than this
volatile bool    z_big = false;
volatile int16_t z_big_c = 0;
volatile long    z_big_e = 0;

// A glitch on A whose two edges both land while interrupts are off (the HX711
// read) collapses into ONE pending INT1. Assuming every call is an edge then
// counts it -- always the same way, so the position creeps. Only count when A
// has actually changed level since the last call.
volatile uint8_t a_last = 0;
const bool ENC_REVERSE = true;         // hand probe 2026-11-02: raw counts rose toward D12
ISR(INT1_vect) {                       // x2 decode: A==B one way, A!=B the other
  uint8_t a = PIND & _BV(3);
  if (a == a_last) return;
  a_last = a;
  bool b = PINB & _BV(0);
  // ENC_REVERSE: the 2026-11-02 rebuild turned the screw the other way round
  // relative to the switches. Counts must still go UP from home (D12) toward D11.
  if (((a != 0) != b) != ENC_REVERSE) { enc++; a_dir = 1; } else { enc--; a_dir = -1; }
}

// wrap a phase difference into -1024..1023 (2048 is a power of two: no division)
inline int16_t wrapPhase(int16_t d) { return ((d + 1024) & 2047) - 1024; }

// Brake = INA=INB=LOW with PWM full: both low sides on. Library speed 0 is a
// COAST, which could carry the shaft through the switch headroom.
inline void brakeNow() { PORTD &= ~(_BV(2) | _BV(4)); OCR1A = 400; }

ISR(PCINT0_vect) {                     // Z = PB2, D11 = PB3, D12 = PB4
  uint8_t pb = PINB, rose = pb & ~pb_prev;
  // At ~130 mm/s Z is high for only a few us, and this ISR often runs behind
  // a queue of INT1 edges -- by then Z has dropped again and no rise is seen
  // (two in a row once tripped "no index pulse for 1.5 revs"). A pin-change
  // interrupt with NO visible change means a pin went and came back; the
  // switches are mechanical and can't do that in microseconds, so it was Z.
  // Only when fast: at low speed Z is wide enough to see, and switch bounce
  // (which only happens near the slow ends) must not pass for an index.
  if (z_fast && pb == pb_prev && !(pb & _BV(2))) rose |= _BV(2);
  pb_prev = pb;
  bool to11 = guard > 0 || (motion_guard && a_dir > 0);
  bool to12 = guard < 0 || (motion_guard && a_dir < 0);
  if (!braked && ((to11 && (pb & _BV(3))) || (to12 && (pb & _BV(4))))) {
    brakeNow(); braked = true; hit_cnt = enc; hit_pin = (to11 && (pb & _BV(3))) ? 11 : 12; guard = 0;
  }
  if (rose & _BV(2)) {
    long e = enc;
    long moved = e - idx_last;
    if (idx_seen) {
      long d = labs(moved);
      if (labs(d - CPR) > 2 && d > 3) idx_bad++;   // +-2: Z-edge jitter at a reversal
    }
    // Direction of this crossing: the last A edge, cross-checked against the
    // net motion since the previous Z (a full rev one way). Disagreement means
    // a reversal near Z -- skip rather than guess.
    int8_t zd = 0;
    if (idx_seen && labs(moved) > 1024 && ((moved > 0) == (a_dir > 0))) zd = a_dir;
    if (zd) {
      uint8_t k = zd > 0 ? 0 : 1;
      int16_t ph = (int16_t)(e & 2047);
      if (z_learn) {
        if (zl_n[k] == 0) z_phase[k] = ph;
        else if (abs(wrapPhase(ph - z_phase[k])) > 2) zl_bad++;
        if (zl_n[k] < 255) zl_n[k]++;
      } else if (z_ok) {
        int16_t c = wrapPhase(z_phase[k] - ph);
        if (abs(c) > Z_MAX_CORR) { z_fault = true; z_fault_d = c; }
        else if (c) {
          enc += c; e += c; z_corr_n++;
          if (abs(c) > z_corr_max) z_corr_max = abs(c);
          if (abs(c) > Z_REPORT) { z_big = true; z_big_c = c; z_big_e = e; }
        }
      }
    }
    idx_seen = true; idx_last = e; idx_n++;
  }
}

long encRead() { noInterrupts(); long e = enc; interrupts(); return e; }
bool lim11() { return PINB & _BV(3); }  // true = OPEN (tripped or broken)
bool lim12() { return PINB & _BV(4); }

// ---- state -----------------------------------------------------------------
bool  armed = false, drv_on = false;
bool  cal_ok = false, arm_after_cal = false;
long  soft_min = 0, soft_max = 0, travel = 0;   // valid only when cal_ok
int8_t motor_sign = 1;                          // learned by the calibration probe
long  counts = 0;
float force_f = 0;                   // lightly smoothed kg
int   speed_now = 0;
int   last_mA = 0, mA_filt = 0;
uint8_t oc_run = 0;   // consecutive over-limit samples
unsigned long last_hx = 0, abs_t0 = 0, abs_phase_t0 = 0;
unsigned long oc_until = 0, oc_last = 0;
uint8_t oc_strikes = 0;
uint8_t abs_phase = 0;               // 0=fwd 1=coast 2=rev 3=coast
bool  abs_active = false, abs_locked = false, in_abs_band = false;
char  line[16]; uint8_t line_n = 0;
const char *pedal_mode = "idle";
float vel_cpms = 0;                  // measured shaft speed, counts/ms (+ = CW)
bool  active_decel = true;
bool  drive_out = false;             // last motor output was DRIVE (not brake/coast)
// Stall boost. The screw is bent over its last ~20 mm (2026-11-02): speed 70
// stalls there (calibration needed command 116 at 47.6 mm). When the rail has
// sat still for STALL_START_MS short of where it is driven, extra command is
// added (1/ms) on top of the normal one, up to BOOST_TOTAL_MAX in all; it bleeds
// off as soon as the shaft moves. Speed limits still come from the UNboosted
// command (active decel), so a bind that suddenly frees can't launch the rail.
// Stuck at full boost for STALL_GIVEUP_MS -> brake and rest STALL_REST_MS,
// rather than sit at ~8 A against something that won't move.
int   drive_boost = 0;
const int BOOST_TOTAL_MAX = 125;
const unsigned long STALL_START_MS = 30, STALL_GIVEUP_MS = 800, STALL_REST_MS = 400;
unsigned long stall_rest_until = 0, stall_n = 0;

// Direct port I/O, interrupts left ON. Blocking them -- for the whole read
// (~150 us) or even per bit with digitalWrite (~12 us) -- collapsed encoder
// edges at seek speed: 31 of 40 index checks bad, travel 144 counts short.
// The HX711 powers down only if SCK stays high > 60 us; the ISRs that can
// stretch a high phase total well under that.  SCK = D5 = PD5, DOUT = D7 = PD7.
long hxRead() {
  unsigned long v = 0;
  for (uint8_t i = 0; i < 24; i++) {
    PORTD |= _BV(5); delayMicroseconds(1);
    v = (v << 1) | ((PIND & _BV(7)) ? 1UL : 0UL);
    PORTD &= ~_BV(5); delayMicroseconds(1);
  }
  PORTD |= _BV(5); delayMicroseconds(1);   // 25th pulse: channel A, gain 128
  PORTD &= ~_BV(5); delayMicroseconds(1);
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (long)v;
}

// EN/DIAG is open-drain with a pull-up on the shield: drive it LOW to disable,
// release it (INPUT) to enable. Never drive it HIGH.
void driver(bool on) {
  guard = 0; md.setM1Speed(0); speed_now = 0; braked = false;
  if (on) pinMode(6, INPUT); else { digitalWrite(6, LOW); pinMode(6, OUTPUT); }
  drv_on = on;
}

// Every pedal-mode motor command goes through here: soft limits, the taper
// toward them, and the direction guard the limit-switch ISR brakes on.
// Returns the speed actually applied.
int motorOut(int s) {
  drive_out = false;
  if (braked || !drv_on) return 0;
  // The guard follows the last direction actually driven and is NOT cleared by
  // a stop: a rail coasting or braking toward a switch is still heading for it.
  // (First pedal run: clearing it here let the rail coast through home unseen.)
  if (s > 0) guard = 1; else if (s < 0) guard = -1;
  if (cal_ok) {
    long p = encRead();
    if (s == 0) {
      // Outside the soft window a stop must BRAKE -- a coast carried the rail
      // ~6 mm past the soft limit. Inside it, 0 stays a coast: ABS relies on it.
      if (p <= soft_min || p >= soft_max) md.setM1Brake(400);
      else md.setM1Speed(0);
      return 0;
    }
    long room = s > 0 ? soft_max - p : p - soft_min;
    if (room <= 0) {            // at the soft limit: hold like an end stop
      md.setM1Brake(400); return 0;
    }
    long cap = room / STOP_K;            // room to stop in, with margin
    if (cap < SPEED_MIN) cap = SPEED_MIN;
    if (cap > speed_max) cap = speed_max;
    if (s >  cap) s =  cap;
    if (s < -cap) s = -cap;
  }
  // Active deceleration. Dropping the PWM only lets the motor COAST down, far
  // slower than the brake: a 400 return overshot park by ~4 mm, and a reversal
  // drove against full back-EMF (13 A -> the "repeated overcurrent" disarm).
  // If the shaft is going faster than this command asks, or the wrong way,
  // brake until it isn't. (Brake current is low-side only: not supply, not CS.)
  if (active_decel) {
    float want = s * V_PER_UNIT;
    if ((vel_cpms > 3 && (s <= 0 || vel_cpms > want * 1.3f + 3)) ||
        (vel_cpms < -3 && (s >= 0 || vel_cpms < want * 1.3f - 3))) {
      md.setM1Brake(400);
      return s;               // the command stands; only the output brakes
    }
  }
  int o = s;
  if (drive_boost > 0 && s != 0) {
    o = s > 0 ? min(s + drive_boost, BOOST_TOTAL_MAX) : max(s - drive_boost, -BOOST_TOTAL_MAX);
    if (abs(o) < abs(s)) o = s;                     // never less than asked
  }
  {
    float v_along = s > 0 ? vel_cpms : -vel_cpms;   // shaft speed in the command's direction
    if (v_along < 0) v_along = 0;
    int lim = (int)(I_CAP_PER_A * i_cap_a + I_CAP_OFFSET + I_CAP_PER_V * v_along);
    if (o >  lim) o =  lim;
    if (o < -lim) o = -lim;
    if (s >  lim) s =  lim;
    if (s < -lim) s = -lim;
  }
  md.setM1Speed(pwm12(o) * motor_sign);
  drive_out = (o != 0);
  return s;                                          // unboosted: what the watchdog judges
}

// Hold position: brake (both low sides on). The screw is self-locking anyway;
// the brake just kills the coast so stops land where they were aimed.
void motorBrake() {
  drive_out = false;
  if (braked || !drv_on) return;
  md.setM1Brake(400);
}

long curvePos(float kg) {
  long top = soft_max - CURVE_TOP_MARGIN;
  if (travel_limit > PARK_COUNTS + TAPER_COUNTS && travel_limit < top) top = travel_limit;
  if (kg <= force_deadband) return PARK_COUNTS;
  long bite = (long)(bite_mm * COUNTS_PER_MM);
  if (bite_mm > 0 && bite > PARK_COUNTS + 341 && bite < top - 1365 &&
      bite_kg > force_deadband + 0.2f && bite_kg < pedal_full_kg - 0.5f) {
    long q = (long)(q_mm * COUNTS_PER_MM);
    if (q_mm > 0 && q > PARK_COUNTS + 341 && q < bite - 341 &&
        q_kg > force_deadband + 0.05f && q_kg < bite_kg - 0.1f) {
      if (kg < q_kg)
        return PARK_COUNTS + (long)((kg - force_deadband) / (q_kg - force_deadband) * (float)(q - PARK_COUNTS));
      if (kg < bite_kg)
        return q + (long)((kg - q_kg) / (bite_kg - q_kg) * (float)(bite - q));
    }
    if (kg < bite_kg)
      return PARK_COUNTS + (long)((kg - force_deadband) / (bite_kg - force_deadband) * (float)(bite - PARK_COUNTS));
    float u = (kg - bite_kg) / (pedal_full_kg - bite_kg);
    if (u > 1) u = 1;
    float t = BITE_A >= 0.999f ? u : (-BITE_A + sqrt(BITE_A * BITE_A + 4 * (1 - BITE_A) * u)) / (2 * (1 - BITE_A));
    return bite + (long)(t * (float)(top - bite));
  }
  float u = (kg - force_deadband) / (pedal_full_kg - force_deadband);
  if (u > 1) u = 1;
  u = pow(u, pedal_gamma);
  return PARK_COUNTS + (long)(u * (float)(top - PARK_COUNTS));
}

// P-control with a hold band. The motor won't turn below SPEED_MIN, so any
// error outside the band gets at least that; inside it we brake and hold, with
// hysteresis so a noisy force can't chatter the motor in and out of hold.
int positionLoop(long pos, long tgt, bool *hold) {
  static bool holding = false;
  long err = tgt - pos, a = labs(err);
  if (a <= (holding ? POS_HOLD_EXIT : POS_HOLD_ENTER)) { holding = true; *hold = true; return 0; }
  holding = false;
  long sp = a / STOP_K;                  // fast as it can go and still stop on target
  if (sp < SPEED_MIN) sp = SPEED_MIN;
  if (sp > speed_max) sp = speed_max;
  return err > 0 ? (int)sp : -(int)sp;
}

void stopMotor(const __FlashStringHelper *why) {
  motorOut(0); speed_now = 0; abs_active = false;
  if (why) { Serial.print(F("STOP: ")); Serial.println(why); }
}

void disarm(const __FlashStringHelper *why) {
  armed = false; arm_after_cal = false; motion_guard = false;
  stopMotor(why); driver(false);
}

void tare() {
  long acc = 0;
  for (uint8_t i = 0; i < 8; i++) {
    unsigned long t0 = millis();
    while (digitalRead(HX_DOUT) == HIGH && millis()-t0 < 400) {}
    acc += hxRead() / 8;
  }
  force_zero = acc; force_f = 0;
  last_hx = millis();
  Serial.print(F("tared, zero=")); Serial.println(force_zero);
}

void arm() {
  if (!cal_ok) {
    Serial.println(F("not calibrated - calibrating first"));
    arm_after_cal = true; startCal();
    return;
  }
  // Tare first: an unloaded offset just over the deadband (0.64 kg once) parks
  // the pedal off its rest, and zero-tracking can't fix it -- it only runs
  // below the deadband. Hands off the cell while arming.
  tare();
  if (drag_ok) {
    drag_fwd_kg = (drag_raw_f - force_zero) / COUNTS_PER_KG;
    drag_bwd_kg = (drag_raw_b - force_zero) / COUNTS_PER_KG;
    // sanity: a sign-swapped or crazy reading must not become a force source
    if (fabs(drag_fwd_kg) > 8 || fabs(drag_bwd_kg) > 8) { drag_fwd_kg = drag_bwd_kg = 0; }
    Serial.print(F("drag fwd ")); Serial.print(drag_fwd_kg); Serial.print(F(" kg  bwd ")); Serial.print(drag_bwd_kg);
    Serial.println(F(" kg (compensated x0.8 while moving)"));
  }
  force_b = 0;
  driver(true); armed = true; motion_guard = true;
  Serial.println(F("ARMED - press the cell"));
}

void help() {
  Serial.println(F("g x c t M=motorTest I<A>=currentCap V<n>=speedMax L<mm>=travelLimit B<mm>,<kg>=padBite y<kg>=hyst w<kg>=damp v<speed>=brakeTest m=mode p<kg>=fullTravel k<gamma> d<kg> a<kg> f<kg>=absFull s<n> h<hz> b=bidir ?"));
  Serial.print(pos_mode ? F("mode POSITION: full travel at ") : F("mode SPEED-FOLLOW (curve: full travel at "));
  Serial.print(pedal_full_kg); Serial.print(F(" kg, gamma ")); Serial.print(pedal_gamma);
  Serial.println(pos_mode ? F("") : F(")"));
  Serial.print(F("deadband ")); Serial.print(force_deadband);
  Serial.print(F("kg  ABS at ")); Serial.print(abs_threshold);
  Serial.print(F("kg  cutoff ")); Serial.print(FORCE_MAX);
  Serial.println(F("kg"));
  if (cal_ok) {
    Serial.print(F("travel ")); Serial.print(travel / COUNTS_PER_MM, 2);
    Serial.print(F(" mm  soft ")); Serial.print(soft_min / COUNTS_PER_MM, 2);
    Serial.print(F("..")); Serial.print(soft_max / COUNTS_PER_MM, 2);
    Serial.println(F(" mm"));
  } else Serial.println(F("not calibrated"));
  Serial.print(F("pad bite ")); Serial.print(bite_mm, 1); Serial.print(F(" mm at ")); Serial.print(bite_kg, 1);
  Serial.print(F(" kg  hyst ")); Serial.print(force_hyst); Serial.print(F(" kg  drag fwd ")); Serial.print(drag_fwd_kg);
  Serial.print(F(" bwd ")); Serial.println(drag_bwd_kg);
  Serial.print(F("travel limit ")); if (travel_limit) Serial.println(travel_limit / COUNTS_PER_MM, 1); else Serial.println(F("off"));
  Serial.print(F("speed_max ")); Serial.print(speed_max);
  Serial.print(F("  current cap ")); Serial.print(i_cap_a, 1); Serial.print(F(" A"));
  Serial.println(z_ok ? F("  Z correction ON") : F("  Z correction OFF"));
}

void handleLine() {
  if (calRunning() && line[0] != 'x') { Serial.println(F("calibrating - x to abort")); return; }
  switch (line[0]) {
    case 'g': arm(); break;
    case 'x': if (calRunning()) abortCal(F("user abort"));
              else disarm(F("disarmed")); break;
    case 'c': disarm(NULL); startCal(); break;
    case '?': help(); break;
    case 'v': speedTest(atoi(line+1)); break;   // braking-distance test (speedtest.ino)
    case 'M': motorTest(); break;                // motor characterisation (motortest.ino)
    case 'I': { float a = atof(line+1);
                if (a >= 2.0f && a <= 16.0f) i_cap_a = a;
                Serial.print(F("current cap ")); Serial.print(i_cap_a, 1);
                Serial.print(F(" A (standstill command ")); Serial.print((int)(I_CAP_PER_A * i_cap_a + I_CAP_OFFSET));
                Serial.println(F(")")); break; }
    case 'Q': { q_mm = atof(line+1);
                char *c = strchr(line, ','); if (c) q_kg = atof(c+1);
                Serial.print(F("knee ")); Serial.print(q_mm, 1); Serial.print(F(" mm at "));
                Serial.print(q_kg, 2); Serial.println(F(" kg")); break; }
    case 'B': { bite_mm = atof(line+1);
                char *c = strchr(line, ','); if (c) bite_kg = atof(c+1);
                Serial.print(F("pad bite ")); if (bite_mm > 0) { Serial.print(bite_mm, 1); Serial.print(F(" mm at "));
                Serial.print(bite_kg, 1); Serial.println(F(" kg")); } else Serial.println(F("off"));
                break; }
    case 'W': damp_cap = atof(line+1); Serial.print(F("damping cap ")); Serial.print(damp_cap); Serial.println(F(" kg")); break;
    case 'w': pedal_damp = constrain(atof(line+1), 0.0, 30.0) / 68.27f;
              Serial.print(F("damping ")); Serial.print(pedal_damp * 68.27f); Serial.println(F(" kg per 100 mm/s")); break;
    case 'y': force_hyst = constrain(atof(line+1), 0.0, 2.0);
              Serial.print(F("force hysteresis ")); Serial.println(force_hyst); break;
    case 'L': { float mm = atof(line+1);
                travel_limit = mm > 0 ? (long)(mm * COUNTS_PER_MM) : 0;
                Serial.print(F("travel limit "));
                if (travel_limit) { Serial.print(mm, 1); Serial.println(F(" mm")); } else Serial.println(F("off"));
                break; }
    case 'U': { float u = atof(line+1);
                if (u >= 10.0f && u <= 24.0f) v_scale = 12.0f / u;
                speed_max = min(speed_max, (int)(SPEED_LIMIT / v_scale));
                Serial.print(F("supply ")); Serial.print(12.0f / v_scale, 1);
                Serial.print(F(" V  max V")); Serial.println((int)(SPEED_LIMIT / v_scale)); break; }
    case 'V': speed_max = constrain(atoi(line+1), SPEED_MIN, (int)(SPEED_LIMIT / v_scale));
              Serial.print(F("speed_max=")); Serial.println(speed_max); break;
    case 'm': stopMotor(NULL); pos_mode = !pos_mode;
              Serial.println(pos_mode ? F("mode POSITION") : F("mode SPEED-FOLLOW")); break;
    case 'p': { float v = atof(line+1);
                if (v > force_deadband + 0.5f) pedal_full_kg = v;
                Serial.print(F("full travel at ")); Serial.print(pedal_full_kg); Serial.println(F(" kg")); break; }
    case 'k': { float v = atof(line+1);
                if (v >= 0.2f && v <= 3.0f) pedal_gamma = v;
                Serial.print(F("gamma=")); Serial.println(pedal_gamma); break; }
    case 's': abs_speed = atoi(line+1);
              Serial.print(F("abs_speed=")); Serial.println(abs_speed); break;
    case 'h': { int hz = atoi(line+1);
                if (hz >= 3 && hz <= 40) {
                  unsigned long half = 500UL / hz;        // ms per half-cycle
                  abs_drive_ms = half * 72 / 100;
                  abs_coast_ms = half - abs_drive_ms;
                  Serial.print(F("ABS ")); Serial.print(hz);
                  Serial.print(F(" Hz  drive=")); Serial.print(abs_drive_ms);
                  Serial.print(F(" coast=")); Serial.println(abs_coast_ms);
                } else Serial.println(F("hz 3..40"));
                break; }
    case 'f': abs_full_kg = atof(line+1);
              Serial.print(F("ABS full at ")); Serial.print(abs_full_kg);
              Serial.println(F(" kg")); break;
    case 'b': abs_bidir = !abs_bidir;
              Serial.print(F("ABS bidirectional=")); Serial.println(abs_bidir); break;
    case 'd': force_deadband = atof(line+1);
              Serial.print(F("deadband=")); Serial.println(force_deadband); break;
    case 'a': abs_threshold  = atof(line+1);
              Serial.print(F("abs at ")); Serial.println(abs_threshold); break;
    case 't': stopMotor(NULL); tare(); break;   // blocks ~0.1 s (up to ~3 s if the HX711 is silent)
  }
}

void setup() {
  md.init(); md.setM1Speed(0);
  digitalWrite(6, LOW); pinMode(6, OUTPUT);   // bridge off before anything else
  // md.init() still claims the unused M2 pins as outputs and enables Timer1
  // PWM on D10 (OC1B). Take them back: D7 = HX711 DOUT, D8/D10 = encoder B/Z.
  TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0));
  pinMode(8, INPUT); pinMode(10, INPUT); pinMode(3, INPUT);
  pinMode(11, INPUT_PULLUP); pinMode(12, INPUT_PULLUP);
  pb_prev = PINB; a_last = PIND & _BV(3);
  EICRA = (EICRA & ~(_BV(ISC11) | _BV(ISC10))) | _BV(ISC10);   // INT1 on CHANGE
  EIFR = _BV(INTF1); EIMSK |= _BV(INT1);
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT4); PCIFR = _BV(PCIF0); PCICR |= _BV(PCIE0);
  Serial.begin(115200);
  pinMode(HX_SCK, OUTPUT); digitalWrite(HX_SCK, LOW);
  pinMode(HX_DOUT, INPUT);
  delay(400);
  Serial.println();
  Serial.println(F("=== active pedal INTERACTIVE SIM ==="));
  Serial.println(F("DISARMED, shaft free. 'g' calibrates (first time) and arms."));
  help();
  Serial.println(F("S,kg,speed,mA,mode,mm,idx_bad,idx_n,zcorr_n/max,lim_motor,lim_home"));
  last_hx = millis();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch=='\n'||ch=='\r') { if (line_n) { line[line_n]=0; handleLine(); line_n=0; } }
    else if (line_n < sizeof(line)-1) line[line_n++] = ch;
  }

  // --- force (non-blocking; a sample every ~11.5 ms) ---
  if (digitalRead(HX_DOUT) == LOW) {
    counts = hxRead(); hx_new = true;
    float kg = (counts - force_zero) / COUNTS_PER_KG;
    // Retuned for 80 SPS. At 10 SPS, alpha 0.5 gave ~200 ms of smoothing; keeping
    // it here would give 23 ms and pass the extra noise the faster rate brings.
    // alpha 0.2 at 87 SPS -> ~57 ms: still 3.5x faster than before, but quieter.
    force_f += 0.2 * (kg - force_f);
    last_hx = millis();
  }
  // The VNH5019 current-sense pin is sampled asynchronously against a 20 kHz
  // PWM, so single reads catch switching spikes that are not real current.
  // Filter, and require the limit to persist, before believing it.
  last_mA = md.getM1CurrentMilliamps();
  mA_filt += (last_mA - mA_filt) / 4;
  long pos = encRead();
  { static unsigned long vt = 0; static long ve = 0;
    unsigned long now = micros();
    if (now - vt >= 2000UL) { vel_cpms = (pos - ve) * 1000.0f / (float)(now - vt); vt = now; ve = pos;
                              z_fast = fabs(vel_cpms) > 20; } }   // >20 counts/ms = ~30 mm/s

  // Slow zero-tracking. Thermal and mechanical drift wanders the baseline by far
  // more than the sensor's own noise, which would otherwise false-trigger the
  // deadband. Only creeps while we are idle and unloaded, so it can never eat a
  // real press: time constant ~10 s, frozen the moment force exceeds the deadband.
  {
    static unsigned long last_zt = 0;
    if (!calRunning() && fabs(force_f) < force_deadband && speed_now == 0 && millis() - last_zt > 100) {
      last_zt = millis();
      force_zero += (long)((counts - force_zero) * 0.01f);
    }
  }

  // --- safety gates ---
  const __FlashStringHelper *fault = NULL;
  if (millis() - last_hx > HX_WATCHDOG_MS) fault = F("load cell timeout");
  else if (force_f > FORCE_MAX)            fault = F("force ceiling");
  else if (drv_on && md.getM1Fault())      fault = F("driver fault");   // D6 reads low while disabled
  if (fault) {
    if (calRunning()) abortCal(fault);
    else if (armed || drv_on) disarm(fault);
  }

  // belt and braces: the ISR brakes on the edge; this catches a missed one
  {
    bool to11 = guard > 0 || (motion_guard && vel_cpms > 0.5f);
    bool to12 = guard < 0 || (motion_guard && vel_cpms < -0.5f);
    if (!braked && ((to11 && lim11()) || (to12 && lim12()))) {
      noInterrupts();
      if (!braked) { brakeNow(); braked = true; hit_cnt = enc; hit_pin = (to11 && lim11()) ? 11 : 12; guard = 0; }
      interrupts();
    }
  }

  if (calRunning()) {
    stepCal(pos);
  } else {
    // A hardware limit in pedal mode means the soft limit was missed: the
    // switches moved, the count slipped, or something else is wrong. Hold the
    // brake briefly, then distrust the calibration until it is redone.
    if (braked) {
      Serial.print(F("STOP: limit switch D")); Serial.print(hit_pin);
      Serial.print(F(" hit at ")); Serial.print(hit_cnt / COUNTS_PER_MM, 2);
      Serial.println(F(" mm - soft limit missed, recalibration required"));
      delay(150);
      braked = false; cal_ok = false;
      disarm(NULL);
    }
    // Encoder envelope: catches a switch that did not trip.
    if (cal_ok && drv_on && (pos < -ENVELOPE_COUNTS || pos > travel + ENVELOPE_COUNTS)) {
      brakeNow(); delay(150);
      Serial.print(F("STOP: outside travel envelope at ")); Serial.print(pos / COUNTS_PER_MM, 2);
      Serial.println(F(" mm - did a limit switch fail? recalibration required"));
      cal_ok = false; disarm(NULL);
    }
    // Encoder-motion watchdog (speed_now is what was applied last pass)
    {
      static unsigned long wd_t = 0, wd_go = 0; static long wd_e = 0; static int8_t wd_dir = 0;
      static bool wd_on = false;
      int8_t d = !drive_out ? 0 : speed_now >= WD_MIN_SPEED ? 1 : speed_now <= -WD_MIN_SPEED ? -1 : 0;
      if (!armed || d == 0 || d != wd_dir) { wd_dir = d; wd_go = millis() + WD_GRACE_MS; wd_on = false; }
      else if (!wd_on) { if ((long)(millis() - wd_go) >= 0) { wd_on = true; wd_t = millis(); wd_e = pos; } }
      else if (millis() - wd_t >= WD_WINDOW_MS) {
        // A stall in the bent part of the screw also stops the counts -- but
        // it draws heavy current (8.5-12.8 A when ABS tripped this at 47 mm,
        // 2026-11-02). A dead encoder or slipped coupling leaves the motor
        // spinning FREE: 1-3 A. Only the low-current case is an encoder fault.
        if ((pos - wd_e) * d < WD_MIN_COUNTS && mA_filt < WD_FREE_MA) {
          brakeNow(); delay(150);
          Serial.print(F("STOP: driving but encoder moved ")); Serial.print((pos - wd_e) * d);
          Serial.println(F(" counts in 10 ms - encoder/coupling fault or jam, recalibration required"));
          cal_ok = false; disarm(NULL);
        }
        wd_t = millis(); wd_e = pos;
      }
    }
    // Encoder integrity. x2 decoding reads B a few us after each A edge; at high
    // speed a late read counts the wrong way. With Z correction a few counts per
    // rev are put back at the next Z; a correction past Z_MAX_CORR, or travel
    // with no Z at all, means the position is genuinely lost.
    // Without Z correction (learning failed), any bad index check stops.
    if (armed) {
      noInterrupts(); bool zf = z_fault; int16_t zfd = z_fault_d; long since = pos - idx_last;
                      unsigned long ib = idx_bad; interrupts();
      static unsigned long bad_seen = 0;
      const __FlashStringHelper *lost = NULL;
      if (zf) lost = F("index correction too large");
      else if (z_ok && labs(since) > 3 * CPR) lost = F("no index pulse for 3 revs");
      else if (!z_ok && ib != bad_seen) lost = F("encoder miscount (no Z correction)");
      bad_seen = ib;
      if (lost) {
        brakeNow(); delay(150);
        Serial.print(F("STOP: ")); Serial.print(lost);
        if (zf) { Serial.print(F(" (")); Serial.print(zfd); Serial.print(F(" counts)")); }
        Serial.print(F(" at ")); Serial.print(pos / COUNTS_PER_MM, 2);
        Serial.println(F(" mm - recalibration required"));
        noInterrupts(); z_fault = false; interrupts();
        cal_ok = false; disarm(NULL);
      }
    }
    { noInterrupts(); bool zb = z_big; int16_t zc = z_big_c; long ze = z_big_e; z_big = false; interrupts();
      if (zb) { Serial.print(F("ZBIG,")); Serial.print(zc); Serial.print(','); Serial.println(ze / COUNTS_PER_MM, 2); } }
    pedalStep(pos);
  }

  static unsigned long last_tx = 0;
  if (millis() - last_tx >= 50) {
    last_tx = millis();
    Serial.print(F("S,")); Serial.print(force_f, 2);
    Serial.print(','); Serial.print(speed_now);
    Serial.print(','); Serial.print(last_mA);
    Serial.print(',');
    if (calRunning()) { Serial.print(F("cal:")); Serial.print(calStateName()); }
    else Serial.print(armed ? pedal_mode : "disarmed");
    Serial.print(','); Serial.print(pos / COUNTS_PER_MM, 2);
    noInterrupts(); unsigned long ib = idx_bad, zc = z_corr_n, zn = idx_n; int16_t zm = z_corr_max; interrupts();
    Serial.print(','); Serial.print(ib);
    Serial.print(','); Serial.print(zn);
    Serial.print(','); Serial.print(zc); Serial.print('/'); Serial.print(zm);
    Serial.print(','); Serial.print(lim11() ? F("OPEN") : F("ok"));
    Serial.print(','); Serial.print(lim12() ? F("OPEN") : F("ok"));
    Serial.print(','); Serial.print(force_est, 2);
    // Throttle pot on A2 (2026-11-15): 2026-11-24 remap: rest 266-272, WOT 646-658.
    // A few counts of dead zone at each end so 0% and 100% are both reliable.
    Serial.print(','); Serial.print(constrain((analogRead(A2) - THR_MIN) * 100.0f / (THR_MAX - THR_MIN), 0.0f, 100.0f), 1);
    Serial.println();
  }
}

void pedalStep(long pos) {
  // Overcurrent is a TRANSIENT here, not a fault: every ABS reversal drives the
  // supply voltage and the back-EMF across the winding at once, which spikes by
  // design. Rest the motor briefly and carry on. Only disarm if the spikes keep
  // coming, which would mean something is actually jammed.
  if (mA_filt > MAX_CURRENT_MA) oc_run++; else oc_run = 0;
  if (oc_run >= 3) {
    oc_run = 0;
    motorOut(0); speed_now = 0;
    oc_until = millis() + OC_BACKOFF_MS;
    if (millis() - oc_last < 2000) {
      if (++oc_strikes >= OC_MAX_STRIKES) {
        oc_strikes = 0; disarm(F("repeated overcurrent - jammed?"));
      }
    } else oc_strikes = 1;
    oc_last = millis();
  }

  // drag-compensated force, then hysteresis
  {
    float sv = vel_cpms / DRAG_V0; if (sv > 1) sv = 1; if (sv < -1) sv = -1;
    float drag = sv > 0 ? drag_fwd_kg * sv : -drag_bwd_kg * sv;   // what the arm adds to the reading
    force_est = force_f - DRAG_GAIN * drag;
    if (force_f >= force_deadband) { float d = pedal_damp * vel_cpms;
      if (damp_cap > 0) d = constrain(d, -damp_cap, damp_cap); force_est -= d; }
    if (force_est < force_deadband) force_b = force_est;
    else if (force_est > force_b + force_hyst) force_b = force_est - force_hyst;
    else if (force_est < force_b - force_hyst) force_b = force_est + force_hyst;
  }
  int target = 0;
  bool hold = false;
  pedal_mode = (millis() < oc_until && in_abs_band) ? "ABS-oc" : "idle";

  if (armed && millis() >= oc_until && force_est >= force_deadband) {
    // Hysteresis: a hand press wobbles either side of the trigger, so without
    // a lower exit point ABS chatters on/off every 100 ms and never sustains.
    if (force_f >= abs_threshold)              in_abs_band = true;
    else if (force_f < abs_threshold * 0.70f)  in_abs_band = false;

    if (in_abs_band && !abs_locked) {
      // ---- ABS: alternate direction with a coast between reversals ----
      pedal_mode = "ABS";
      if (!abs_active) { abs_active = true; abs_t0 = millis(); abs_phase = 0; abs_phase_t0 = millis(); }
      if (millis() - abs_t0 > ABS_MAX_MS) {
        abs_locked = true;                   // cool-down until force is released
        Serial.println(F("ABS burst limit - release to reset"));
      } else {
        unsigned long ph = millis() - abs_phase_t0;
        unsigned long len = (abs_phase & 1) ? abs_coast_ms : abs_drive_ms;
        if (ph >= len) { abs_phase = (abs_phase + 1) & 3; abs_phase_t0 = millis(); }
        // Pulse amplitude scales with pedal force: a light flutter just over the
        // trigger, building to full violence at abs_full_kg. Real ABS modulates
        // how hard it hammers, not just whether it does.
        float t = (force_f - abs_threshold) / (abs_full_kg - abs_threshold);
        if (t < 0) t = 0; else if (t > 1) t = 1;
        int amp = ABS_SPEED_MIN + (int)(t * (float)(abs_speed - ABS_SPEED_MIN));
        int pulse;
        if      (abs_phase == 0) pulse =  amp;
        else if (abs_phase == 2) pulse =  abs_bidir ? -amp : amp;
        else                     pulse =  0;
        if (pos_mode) {
          // Pulse ON TOP of the position loop. Pulses alone replaced it: the
          // rail froze wherever ABS caught it (38 mm at 24 kg, curve said 50).
          bool h = false;
          int base = positionLoop(pos, curvePos(force_b), &h);
          target = constrain(base + pulse, -speed_max, speed_max);
        } else target = pulse;
        // Active decel stays ON here: with it off, a 400 stomp into ABS coasted
        // through the end soft limit. Reversals now brake, then drive.
        speed_now = motorOut(target);        // no slew: ABS must be crisp
      }
    } else if (pos_mode) {
      // ---- pedal: position set by force through the curve ----
      abs_active = false;
      target = positionLoop(pos, curvePos(force_b), &hold);
      pedal_mode = hold ? "hold" : "pedal";
    } else {
      // ---- follow: speed proportional to force ----
      pedal_mode = "follow";
      abs_active = false;
      float span = abs_threshold - force_deadband;
      float f = (force_est - force_deadband) / span;
      if (f > 1) f = 1;
      target = SPEED_MIN + (int)(f * (speed_max - SPEED_MIN));
    }
  } else {
    abs_active = false;
    if (force_est < force_deadband) { abs_locked = false; in_abs_band = false; }
    // released -> back to park
    if (armed && millis() >= oc_until) {
      if (pos_mode) {
        target = positionLoop(pos, PARK_COUNTS, &hold);
        pedal_mode = hold ? "rest" : "return";
      } else if (pos > PARK_COUNTS + PARK_EPS) {
        // the soft-limit taper in motorOut does the slowing; only the stop matters
        long room = pos - PARK_COUNTS;
        int cap = SPEED_MIN + (int)((long)(RETURN_SPEED - SPEED_MIN) * min(room, TAPER_COUNTS) / TAPER_COUNTS);
        target = -cap; pedal_mode = "return";
      } else {
        // At park: brake rather than coast. A coast after the return let it
        // land anywhere from 1.3 to 2.0 mm.
        hold = true; pedal_mode = "rest";
      }
    }
  }

  // ---- stall boost (see drive_boost) ----
  {
    static unsigned long bt = 0, still_t = 0, maxed_t = 0; static int8_t bdir = 0;
    unsigned long now = millis(), dtb = now - bt; bt = now;
    bool pedal_drive = armed && !(abs_active && !abs_locked) && target != 0;
    int8_t d = target > 0 ? 1 : -1;
    if (now < stall_rest_until) { target = 0; hold = true; drive_boost = 0; pedal_mode = "stall"; }
    else if (!pedal_drive || d != bdir) { drive_boost = 0; still_t = now; maxed_t = 0; bdir = pedal_drive ? d : 0; }
    else {
      float va = d > 0 ? vel_cpms : -vel_cpms;
      if (va > 2.0f) still_t = now;
      if (now - still_t >= STALL_START_MS) drive_boost += dtb;
      else if (va > 5.0f) drive_boost -= 2 * dtb;
      drive_boost = constrain(drive_boost, 0, BOOST_TOTAL_MAX);
      if (drive_boost >= BOOST_TOTAL_MAX - abs(target)) {
        if (!maxed_t) maxed_t = now;
        else if (now - maxed_t > STALL_GIVEUP_MS) {
          stall_rest_until = now + STALL_REST_MS; stall_n++; drive_boost = 0; maxed_t = 0;
          target = 0; hold = true;
          Serial.print(F("STALL,")); Serial.println(pos / COUNTS_PER_MM, 2);
        }
      } else maxed_t = 0;
    }
  }

  // slew-limit everything except ABS (which sets speed_now directly above)
  if (!(abs_active && !abs_locked)) {
    static unsigned long last_slew = 0;
    unsigned long dt = millis() - last_slew;
    if (dt) {
      last_slew = millis();
      // Slew limits speeding UP only (that is what draws inrush). Slowing down
      // is immediate: slewed at 2/ms, the return from 260 took ~100 ms = ~5 mm
      // to stop and overran park into the home end.
      float step = SLEW_PER_MS * dt;
      if ((long)speed_now * target < 0) speed_now = 0;      // reversing: stop first
      if (abs(target) <= abs(speed_now)) speed_now = target; // slowing: at once
      else if (target > 0) speed_now = min(target, speed_now + (int)step);
      else                 speed_now = max(target, speed_now - (int)step);
      speed_now = motorOut(speed_now);
      if (hold && speed_now == 0) motorBrake();
    }
  }
}
