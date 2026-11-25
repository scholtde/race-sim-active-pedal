/*
 * Active pedal -- LIMIT-SWITCH CALIBRATION.
 *
 * Motor and encoder sit directly on opposite ends of the TR12x3 lead screw:
 * 2048 counts/rev, 3 mm/rev -> 682.7 counts/mm. The switches will be moved as
 * the project goes, so travel is MEASURED here, never assumed.
 *
 * 'c' runs:
 *   1. probe   - tiny bounded nudge to learn which way +speed turns the shaft
 *   2. seek    - fast to the encoder-end switch (home), brake on the trip
 *   3. back off, re-approach slowly -> trip count = position 0
 *   4. seek, back off, re-approach the motor-end switch -> end of travel
 *   5. return and park 2 mm off home
 * Overtravel past each trip is reported: that is the braking distance, and it
 * has to stay inside the headroom before the shaft reaches the bearing.
 *
 * Encoder: A=D3 (INT1, x2), B=D8, Z=D10.  Limits (NC to GND, pull-ups):
 * D11 = motor end (CW, counts up), D12 = encoder end (CCW, home).
 * A limit trip toward the moving direction BRAKES in the ISR (INA=INB=0,
 * PWM full = both low sides on). Coasting could carry the shaft into the
 * bearing; library speed 0 is a coast, not a brake.
 *
 * Boots with the bridge DISABLED (EN/DIAG D6 held low): shaft free by hand.
 * Commands:  c = calibrate   x = abort   e/d = driver enable/disable   ? = help
 */
#include <DualVNH5019MotorShield.h>
#include "cal_state.h"   // the enum must precede Arduino's generated prototypes
DualVNH5019MotorShield md;

const long  CPR = 2048;
const float COUNTS_PER_MM = CPR / 3.0;
const int   PROBE_SPEED = 100, SEEK_SPEED = 150, APPROACH_SPEED = 90;
const long  PROBE_COUNTS    = 120;     // nudge size: ~0.2 mm
const long  BACKOFF_COUNTS  = 1200;    // switch re-closes ~500 counts back
const long  MAX_SEEK_COUNTS = 60000;   // full travel measured ~36k
const long  PARK_COUNTS     = 1365;    // 2 mm off the home trip point
const long  SLOWDOWN_COUNTS = 3000;    // drop to approach speed this far out
const unsigned long PHASE_TIMEOUT_MS = 10000, STALL_MS = 300;
const long  STALL_COUNTS    = 20;
const long  WRONG_WAY_COUNTS = 300;
const int   MAX_CURRENT_MA  = 14000;
const unsigned long RAMP_MS = 400;     // full-scale ramp; less trips on inrush

// ---- ISR state ---------------------------------------------------------------
volatile long enc = 0;
volatile int8_t  guard = 0;            // +1 moving CW: D11 brakes; -1 CCW: D12 brakes
volatile bool    braked = false;
volatile long    hit_cnt = 0;
volatile uint8_t hit_pin = 0;          // 11 or 12
volatile uint8_t pb_prev = 0;
volatile long idx_last = 0;
volatile bool idx_seen = false;
volatile unsigned long idx_n = 0, idx_bad = 0;

// A glitch on A whose two edges both land while interrupts are off (the HX711
// read) collapses into ONE pending INT1. Assuming every call is an edge then
// counts it -- always the same way, so the position creeps. Only count when A
// has actually changed level since the last call.
volatile uint8_t a_last = 0;
ISR(INT1_vect) {                       // x2 decode: A==B one way, A!=B the other
  uint8_t a = PIND & _BV(3);
  if (a == a_last) return;
  a_last = a;
  bool b = PINB & _BV(0);
  if ((a != 0) != b) enc++; else enc--;
}

inline void brakeNow() { PORTD &= ~(_BV(2) | _BV(4)); OCR1A = 400; }

ISR(PCINT0_vect) {                     // Z = PB2, D11 = PB3, D12 = PB4
  uint8_t pb = PINB, rose = pb & ~pb_prev;
  pb_prev = pb;
  if (!braked && ((guard > 0 && (pb & _BV(3))) || (guard < 0 && (pb & _BV(4))))) {
    brakeNow(); braked = true; hit_cnt = enc; hit_pin = guard > 0 ? 11 : 12; guard = 0;
  }
  if (rose & _BV(2)) {
    long e = enc;
    if (idx_seen) {
      long d = labs(e - idx_last);
      if (labs(d - CPR) > 2 && d > 3) idx_bad++;   // +-2: Z-edge jitter at a reversal
    }
    idx_seen = true; idx_last = e; idx_n++;
  }
}

long encRead() { noInterrupts(); long e = enc; interrupts(); return e; }
bool lim11() { return PINB & _BV(3); }  // true = OPEN (tripped or broken)
bool lim12() { return PINB & _BV(4); }

// ---- motion ------------------------------------------------------------------
bool  drv_on = false;
int8_t motor_sign = 1;                 // +speed -> counts up? learned by the probe
int   target = 0, speed_now = 0;       // in ENCODER direction (+ = CW)
int   mA_filt = 0; uint8_t oc_run = 0;

void driver(bool on) {
  guard = 0; target = speed_now = 0; md.setM1Speed(0); braked = false;
  if (on) pinMode(6, INPUT); else { digitalWrite(6, LOW); pinMode(6, OUTPUT); }
  drv_on = on;
  Serial.println(on ? F("driver ENABLED") : F("driver DISABLED (outputs off, shaft free)"));
}

const char *const ST_NAME[] = { "idle", "probe", "settle", "seek_home", "back_home",
  "appr_home", "back_home2", "seek_end", "back_end", "appr_end", "return", "done" };
St st = S_IDLE, settle_next = S_IDLE, settle_from = S_IDLE;
unsigned long phase_t0 = 0, win_t = 0, closed_t = 0;
long entry_e = 0, win_e = 0, settle_e = 0;
long home_raw = 0, end_cnt = 0, ot_home = 0, ot_end = 0, seek_hit = 0;
int8_t move_dir = 0;
bool slow = false;

void abortCal(const __FlashStringHelper *why) {
  guard = 0; brakeNow(); braked = true;
  delay(200);
  Serial.print(F("C,ABORT,")); Serial.print(ST_NAME[st]); Serial.print(','); Serial.println(why);
  st = S_IDLE; driver(false);
}

void startMove(St s, int8_t dir, int spd, bool guarded) {
  st = s; move_dir = dir;
  speed_now = 0; target = dir * spd;
  braked = false; md.setM1Speed(0);
  guard = guarded ? dir : 0;
  long e = encRead();
  entry_e = win_e = e; phase_t0 = win_t = millis(); closed_t = 0;
  // The ISR only sees edges. A switch that is already open toward the move
  // would never fire it, so refuse here.
  if (guarded && ((dir > 0 && lim11()) || (dir < 0 && lim12()))) {
    brakeNow(); braked = true; hit_cnt = e; hit_pin = dir > 0 ? 11 : 12; guard = 0;
  }
}

void goSettle(St next) {
  guard = 0; brakeNow(); braked = true; target = speed_now = 0;
  settle_from = st; settle_next = next; st = S_SETTLE;
  settle_e = encRead(); phase_t0 = win_t = millis();
}

void say(const __FlashStringHelper *what, long v) {
  Serial.print(F("C,")); Serial.print(what); Serial.print('=');
  Serial.print(v); Serial.print(F(" (")); Serial.print(v / COUNTS_PER_MM, 2);
  Serial.println(F(" mm)"));
}

void enter(St s);

// Back off a switch until it has re-closed (debounced) and we are a margin clear.
void stepBackoff(long e, bool (*lim)(), St next) {
  if (lim()) closed_t = 0; else if (!closed_t) closed_t = millis();
  if (closed_t && millis() - closed_t > 30 && labs(e - entry_e) >= BACKOFF_COUNTS)
    goSettle(next);
}

void enter(St s) {
  switch (s) {
    case S_SEEK_HOME: startMove(S_SEEK_HOME, -1, SEEK_SPEED, true); break;
    case S_BACK_HOME: startMove(S_BACK_HOME, +1, APPROACH_SPEED, true); break;
    case S_APPR_HOME: startMove(S_APPR_HOME, -1, APPROACH_SPEED, true); break;
    case S_BACK_HOME2:startMove(S_BACK_HOME2, +1, APPROACH_SPEED, true); break;
    case S_SEEK_END:  startMove(S_SEEK_END, +1, SEEK_SPEED, true); break;
    case S_BACK_END:  startMove(S_BACK_END, -1, APPROACH_SPEED, true); break;
    case S_APPR_END:  startMove(S_APPR_END, +1, APPROACH_SPEED, true); break;
    case S_RETURN:    startMove(S_RETURN, -1, SEEK_SPEED, true); slow = false; break;
    case S_DONE: {
      long travel = end_cnt;
      Serial.println(F("C,=== calibration done ==="));
      say(F("travel home->end"), travel);
      say(F("overtravel past home trip"), ot_home);
      say(F("overtravel past end trip"), ot_end);
      Serial.print(F("C,motor_sign=")); Serial.println(motor_sign);
      noInterrupts(); unsigned long ib = idx_bad, in = idx_n; interrupts();
      Serial.print(F("C,index pulses=")); Serial.print(in);
      Serial.print(F(" bad=")); Serial.println(ib);
      st = S_IDLE; driver(false);
      break; }
    default: st = s;
  }
}

void stepSettle(long e) {
  if (e != settle_e) { settle_e = e; win_t = millis(); }
  if (millis() - phase_t0 > 2000) { abortCal(F("never stopped")); return; }
  if (millis() - win_t < 60) return;
  // stopped: account for what this stop meant, then release the brake
  switch (settle_from) {
    case S_PROBE:
      Serial.print(F("C,probe: +speed moves counts "));
      Serial.println(motor_sign > 0 ? F("UP (CW)") : F("DOWN (CCW)"));
      break;
    case S_SEEK_HOME: ot_home = seek_hit - e;
      say(F("home trip (fast)"), seek_hit); say(F("braking overtravel"), ot_home); break;
    case S_APPR_HOME: {
      say(F("home trip (slow) raw"), home_raw);
      noInterrupts(); enc -= home_raw; idx_last -= home_raw; interrupts();
      Serial.println(F("C,position zeroed at home trip"));
      break; }
    case S_SEEK_END: ot_end = e - seek_hit;
      say(F("end trip (fast)"), seek_hit); say(F("braking overtravel"), ot_end); break;
    case S_APPR_END: say(F("end trip (slow)"), end_cnt); break;
    default: break;
  }
  braked = false; md.setM1Speed(0);
  if (settle_next == S_SEEK_HOME && lim12()) settle_next = S_BACK_HOME;   // already home
  enter(settle_next);
}

void stepMotion(long e) {
  // a trip from the ISR (or found already open at the start of a move)
  if (braked) {
    uint8_t want = 0;
    if (st == S_SEEK_HOME || st == S_APPR_HOME) want = 12;
    if (st == S_SEEK_END  || st == S_APPR_END)  want = 11;
    if (hit_pin != want) { abortCal(hit_pin == 11 ? F("unexpected motor-end limit")
                                                    : F("unexpected home limit")); return; }
    if (st == S_SEEK_HOME) { seek_hit = hit_cnt; goSettle(S_BACK_HOME); }
    else if (st == S_APPR_HOME) { home_raw = hit_cnt; goSettle(S_BACK_HOME2); }
    else if (st == S_SEEK_END)  { seek_hit = hit_cnt; goSettle(S_BACK_END); }
    else                      { end_cnt = hit_cnt; goSettle(S_RETURN); }
    return;
  }
  switch (st) {
    case S_BACK_HOME: stepBackoff(e, lim12, S_APPR_HOME); break;
    case S_BACK_HOME2:stepBackoff(e, lim12, S_SEEK_END);  break;
    case S_BACK_END:  stepBackoff(e, lim11, S_APPR_END);  break;
    case S_RETURN:
      if (!slow && e <= PARK_COUNTS + SLOWDOWN_COUNTS) { slow = true; target = -APPROACH_SPEED; }
      if (e <= PARK_COUNTS) { goSettle(S_DONE); }
      break;
    default: break;
  }
  if (st == S_SETTLE) return;

  if (millis() - phase_t0 > PHASE_TIMEOUT_MS) { abortCal(F("phase timeout")); return; }
  if (labs(e - entry_e) > MAX_SEEK_COUNTS)   { abortCal(F("switch not found in max travel")); return; }
  if (move_dir * (e - entry_e) < -WRONG_WAY_COUNTS) { abortCal(F("moving the wrong way")); return; }
  if (millis() - win_t >= STALL_MS) {
    if (speed_now == target && labs(e - win_e) < STALL_COUNTS) { abortCal(F("stall")); return; }
    win_t = millis(); win_e = e;
  }
}

void stepProbe(long e) {
  long d = e - entry_e;
  if (labs(d) >= PROBE_COUNTS) {
    motor_sign = d > 0 ? 1 : -1;
    goSettle(S_SEEK_HOME);
  } else if (millis() - phase_t0 > 800) abortCal(F("probe: no motion - PSU off?"));
}

void startCal() {
  if (!drv_on) driver(true);
  delay(5);
  if (md.getM1Fault()) { abortCal(F("driver fault at start")); return; }
  noInterrupts(); idx_bad = 0; idx_n = 0; idx_seen = false; interrupts();
  Serial.println(F("C,=== calibration start ==="));
  motor_sign = 1;              // provisional: the probe runs in raw motor terms
  startMove(S_PROBE, +1, PROBE_SPEED, false);   // bounded by count, not by switches
}

void setup() {
  md.init(); md.setM1Speed(0);
  digitalWrite(6, LOW); pinMode(6, OUTPUT);   // bridge off before anything else
  // md.init() still claims the unused M2 pins and puts Timer1 PWM on D10.
  TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0));
  pinMode(7, INPUT); pinMode(8, INPUT); pinMode(10, INPUT); pinMode(3, INPUT);
  digitalWrite(5, LOW); pinMode(5, OUTPUT);   // HX711 SCK: not used here, keep it parked low
  pinMode(11, INPUT_PULLUP); pinMode(12, INPUT_PULLUP);
  pb_prev = PINB; a_last = PIND & _BV(3);
  EICRA = (EICRA & ~(_BV(ISC11) | _BV(ISC10))) | _BV(ISC10);
  EIFR = _BV(INTF1); EIMSK |= _BV(INT1);
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT4); PCIFR = _BV(PCIF0); PCICR |= _BV(PCIE0);
  Serial.begin(115200);
  Serial.println(F("\n=== limit-switch calibration: c x e d ? ==="));
  driver(false);
  Serial.println(F("E,counts,mm,lim_motor,lim_home,speed,mA,state"));
}

void loop() {
  static char line[12]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { line[n] = 0; n = 0;
        if      (line[0] == 'x') { if (st != S_IDLE) abortCal(F("user abort")); else driver(false); }
        else if (line[0] == 'c') { if (st == S_IDLE) startCal(); }
        else if (line[0] == 'e') { if (st == S_IDLE) driver(true); }
        else if (line[0] == 'd') { if (st == S_IDLE) driver(false); }
        else Serial.println(F("c x e d"));
      }
    } else if (n < sizeof(line) - 1) line[n++] = c;
  }

  long e = encRead();
  int mA = md.getM1CurrentMilliamps();
  mA_filt += (mA - mA_filt) / 4;

  if (st != S_IDLE && st != S_SETTLE) {
    if (drv_on && md.getM1Fault()) abortCal(F("driver fault"));
    else {
      // overcurrent only counts while driving; a brake spike is expected
      if (!braked && mA_filt > MAX_CURRENT_MA) oc_run++; else oc_run = 0;
      if (oc_run >= 3) abortCal(F("overcurrent"));
    }
  }
  // belt and braces: the ISR brakes on the edge; this catches a missed one
  if (!braked && ((guard > 0 && lim11()) || (guard < 0 && lim12()))) {
    noInterrupts();
    if (!braked) { brakeNow(); braked = true; hit_cnt = enc; hit_pin = guard > 0 ? 11 : 12; guard = 0; }
    interrupts();
  }

  if      (st == S_PROBE)  stepProbe(e);
  else if (st == S_SETTLE) stepSettle(e);
  else if (st != S_IDLE)   stepMotion(e);

  // ramp, unless braked
  static unsigned long last_ramp = 0;
  unsigned long dt = millis() - last_ramp;
  if (dt) {
    last_ramp = millis();
    if (!braked && st != S_IDLE && st != S_SETTLE) {
      int step = max(1, (int)(400UL * dt / RAMP_MS));
      if      (speed_now < target - step) speed_now += step;
      else if (speed_now > target + step) speed_now -= step;
      else                                speed_now  = target;
      md.setM1Speed(speed_now * motor_sign);
    }
  }

  static unsigned long last_tx = 0;
  if (millis() - last_tx >= (st == S_IDLE ? 500UL : 100UL)) {
    last_tx = millis();
    Serial.print(F("E,")); Serial.print(e);
    Serial.print(','); Serial.print(e / COUNTS_PER_MM, 2);
    Serial.print(','); Serial.print(lim11() ? F("OPEN") : F("ok"));
    Serial.print(','); Serial.print(lim12() ? F("OPEN") : F("ok"));
    Serial.print(','); Serial.print(speed_now);
    Serial.print(','); Serial.print(mA);
    Serial.print(','); Serial.println(ST_NAME[st]);
  }
}
