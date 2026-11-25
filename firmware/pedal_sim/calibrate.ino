/*
 * Limit-switch calibration (ported from the standalone pedal_cal sketch).
 *
 *   1. probe   - tiny bounded nudge to learn which way +speed turns the shaft
 *   2. seek    - fast to the encoder-end switch (home), brake on the trip
 *   3. back off, re-approach slowly -> trip count = position 0
 *   4. seek, back off, re-approach the motor-end switch -> end of travel
 *   5. return and park 2 mm off home; soft limits sit SOFT_MARGIN inside
 * Overtravel past each fast trip is the braking distance; it has to stay
 * inside the headroom before the shaft reaches the bearing.
 *
 * Measured 2026-10-17: travel 53.07 mm +-0.03, overtravel ~0.76 mm at seek
 * speed 150 (~41 mm/s), motor_sign +1. The home lever was then bent and
 * straightened: its trip moved ~1.2 mm toward the bearing, travel now ~54.3 mm.
 */
// Seek 100, not 150: braking overtravel goes roughly with speed squared
// (0.77 mm at 150 -> ~0.35 mm at 100), and the lever overtravel is small.
// 2026-11-02 rebuild: lever overrun only 1.43 mm (D12) / 1.48 mm (D11) by hand.
// Seek slowly (~17 mm/s); braking from 100 was 0.45-0.51 mm, 70 should be ~0.3.
const int   PROBE_SPEED = 100, SEEK_SPEED = 70, APPROACH_SPEED = 70;
const long  PROBE_COUNTS     = 120;    // nudge size: ~0.2 mm
// Headroom past each trip before the roller lever bottoms out (measured by hand
// 2026-10-17): home 1.38 mm, motor end >= 1.48 mm. The LEVER is what breaks,
// long before the bearing. Braking past a trip must stay under half of that.
const long  OVERTRAVEL_MAX   = 478;    // 0.7 mm
const long  BACKOFF_COUNTS   = 1200;   // switch re-closes ~500 counts back
const long  MAX_SEEK_COUNTS  = 60000;  // full travel measured ~36k
const long  SLOWDOWN_COUNTS  = 3000;   // drop to approach speed this far out
const unsigned long PHASE_TIMEOUT_MS = 10000, STALL_MS = 300;
const long  STALL_COUNTS     = 20;
const long  WRONG_WAY_COUNTS = 300;
const unsigned long RAMP_MS  = 400;    // full-scale ramp; less trips on inrush
// The screw is bent over its last ~15 mm toward D11 (2026-11-02): at seek 70
// (~2 V) current rose 2 -> 5.3 A from 33 mm and it stalled at 43.8 mm. So the
// seek holds a SPEED, not a duty: when the shaft drops below 60% of the
// free-running speed, drive is boosted 1 unit/ms, capped by the motor model
// at CAL_I_CAP_A. It decays again once the shaft is back up to speed, so the
// rail still meets the switch at a crawl.
const float CAL_I_CAP_A = 8.0;
int  c_boost = 0;
// 2026-11-03: with the cell rod too short the pedal hits its back stop before
// D12 trips; seek home then crushed it to 22 kg and stalled at 10.6 A. Homing
// on force was tried and false-triggered on arm drag/preload swings (4-6 kg),
// so force is only a guard: a 10 kg rise during seek home aborts.
const float HOME_F_ABORT_KG = 10.0;
float c_fmin = 0;

const char *const ST_NAME[] = { "idle", "probe", "settle", "seek_home", "back_home",
  "appr_home", "back_home2", "seek_end", "back_end", "appr_end", "return", "done" };
St cst = S_IDLE, settle_next = S_IDLE, settle_from = S_IDLE;
unsigned long phase_t0 = 0, win_t = 0, closed_t = 0;
long entry_e = 0, win_e = 0, settle_e = 0;
long home_raw = 0, end_cnt = 0, ot_home = 0, ot_end = 0, seek_hit = 0;
int8_t move_dir = 0;
int  c_target = 0, c_speed = 0;        // in ENCODER direction (+ = CW)
bool c_slow = false;
uint8_t c_oc_run = 0;

int  c_boost_peak = 0; long c_boost_at = 0;
// pedal-arm drag: raw cell average while moving at seek speed in the free part
// of the screw (3..25 mm), each way. Turned into kg after the tare on arm.
long long dacc_f = 0, dacc_b = 0; int dn_f = 0, dn_b = 0;
// at the cap when standing still (the cap grows with speed)
bool c_boost_maxed() { return abs(c_target) + c_boost >= (int)(I_CAP_PER_A * CAL_I_CAP_A + I_CAP_OFFSET) - 2; }
bool calRunning() { return cst != S_IDLE; }
const char *calStateName() { return ST_NAME[cst]; }

void abortCal(const __FlashStringHelper *why) {
  noInterrupts(); z_learn = false; z_ok = false; interrupts();
  guard = 0; brakeNow(); braked = true;
  delay(200);
  Serial.print(F("C,ABORT,")); Serial.print(ST_NAME[cst]); Serial.print(','); Serial.println(why);
  cst = S_IDLE; cal_ok = false; armed = false; arm_after_cal = false;
  driver(false);
}

void startMove(St s, int8_t dir, int spd, bool guarded) {
  cst = s; move_dir = dir;
  c_speed = 0; c_target = dir * spd; c_boost = 0;
  braked = false; md.setM1Speed(0);
  guard = guarded ? dir : 0;
  long e = encRead();
  entry_e = win_e = e; phase_t0 = win_t = millis(); closed_t = 0;
  if (s == S_SEEK_HOME) c_fmin = 1e6;
  // The ISR only sees edges. A switch that is already open toward the move
  // would never fire it, so treat it as hit right here.
  if (guarded && ((dir > 0 && lim11()) || (dir < 0 && lim12()))) {
    brakeNow(); braked = true; hit_cnt = e; hit_pin = dir > 0 ? 11 : 12; guard = 0;
  }
}

void goSettle(St next) {
  guard = 0; brakeNow(); braked = true; c_target = c_speed = 0;
  settle_from = cst; settle_next = next; cst = S_SETTLE;
  settle_e = encRead(); phase_t0 = win_t = millis();
}

void say(const __FlashStringHelper *what, long v) {
  Serial.print(F("C,")); Serial.print(what); Serial.print('=');
  Serial.print(v); Serial.print(F(" (")); Serial.print(v / COUNTS_PER_MM, 2);
  Serial.println(F(" mm)"));
}

// Back off a switch until it has re-closed (debounced) and we are a margin clear.
void stepBackoff(long e, bool (*lim)(), St next) {
  if (lim()) closed_t = 0; else if (!closed_t) closed_t = millis();
  if (closed_t && millis() - closed_t > 30 && labs(e - entry_e) >= BACKOFF_COUNTS)
    goSettle(next);
}

void finishCal() {
  noInterrupts(); z_learn = false;
  bool zgood = zl_n[0] >= 3 && zl_n[1] >= 3 && zl_bad == 0;
  z_ok = zgood; z_fault = false; z_corr_n = 0; z_corr_max = 0;
  int16_t p0 = z_phase[0], p1 = z_phase[1]; uint8_t n0 = zl_n[0], n1 = zl_n[1], nb = zl_bad;
  interrupts();
  Serial.print(F("C,Z phase CW=")); Serial.print(p0); Serial.print(F(" (n=")); Serial.print(n0);
  Serial.print(F(") CCW=")); Serial.print(p1); Serial.print(F(" (n=")); Serial.print(n1);
  Serial.print(F(") inconsistent=")); Serial.print(nb);
  Serial.println(zgood ? F(" -> Z correction ON") : F(" -> Z correction OFF"));
  if (dn_f >= 20 && dn_b >= 20) {
    drag_raw_f = (long)(dacc_f / dn_f); drag_raw_b = (long)(dacc_b / dn_b); drag_ok = true;
  }
  Serial.print(F("C,drag samples fwd=")); Serial.print(dn_f); Serial.print(F(" bwd=")); Serial.println(dn_b);
  travel = end_cnt;
  soft_min = SOFT_MARGIN; soft_max = end_cnt - SOFT_MARGIN;
  cal_ok = soft_max > soft_min + TAPER_COUNTS;   // refuse a uselessly short travel
  Serial.println(F("C,=== calibration done ==="));
  say(F("travel home->end"), travel);
  say(F("overtravel past home trip"), ot_home);
  say(F("overtravel past end trip"), ot_end);
  say(F("soft min"), soft_min); say(F("soft max"), soft_max);
  Serial.print(F("C,motor_sign=")); Serial.println(motor_sign);
  Serial.print(F("C,peak boost=")); Serial.print(c_boost_peak);
  Serial.print(F(" at ")); Serial.print(c_boost_at / COUNTS_PER_MM, 2); Serial.println(F(" mm"));
  noInterrupts(); unsigned long ib = idx_bad, in = idx_n; interrupts();
  Serial.print(F("C,index pulses=")); Serial.print(in);
  Serial.print(F(" bad=")); Serial.println(ib);
  cst = S_IDLE; braked = false; md.setM1Speed(0); c_speed = 0;
  if (!cal_ok) Serial.println(F("C,travel too short - not calibrated"));
  if (cal_ok && arm_after_cal) { arm_after_cal = false; arm(); }
  else driver(false);
}

void enterCal(St s) {
  switch (s) {
    case S_SEEK_HOME: startMove(S_SEEK_HOME, -1, SEEK_SPEED, true); break;
    case S_BACK_HOME: startMove(S_BACK_HOME, +1, APPROACH_SPEED, true); break;
    case S_APPR_HOME: startMove(S_APPR_HOME, -1, APPROACH_SPEED, true); break;
    case S_BACK_HOME2:startMove(S_BACK_HOME2, +1, APPROACH_SPEED, true); break;
    case S_SEEK_END:  startMove(S_SEEK_END, +1, SEEK_SPEED, true);
                      noInterrupts(); z_learn = true; interrupts(); break;   // CW phase
    case S_BACK_END:  startMove(S_BACK_END, -1, APPROACH_SPEED, true); break;
    case S_APPR_END:  startMove(S_APPR_END, +1, APPROACH_SPEED, true); break;
    case S_RETURN:    startMove(S_RETURN, -1, SEEK_SPEED, true); c_slow = false; break;
    case S_DONE:      finishCal(); break;
    default: cst = s;
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
      say(F("home trip (fast)"), seek_hit); say(F("braking overtravel"), ot_home);
      if (ot_home > OVERTRAVEL_MAX) { abortCal(F("braking overtravel past home exceeds 0.7 mm - lever at risk")); return; }
      break;
    case S_APPR_HOME:
      say(F("home trip (slow) raw"), home_raw);
      noInterrupts(); enc -= home_raw; idx_last -= home_raw;
                      zl_n[0] = zl_n[1] = 0; zl_bad = 0; z_ok = false; interrupts();
      Serial.println(F("C,position zeroed at home trip"));
      break;
    case S_SEEK_END: ot_end = e - seek_hit;
      say(F("end trip (fast)"), seek_hit); say(F("braking overtravel"), ot_end);
      if (ot_end > OVERTRAVEL_MAX) { abortCal(F("braking overtravel past end exceeds 0.7 mm - lever at risk")); return; }
      break;
    case S_APPR_END: say(F("end trip (slow)"), end_cnt); break;
    default: break;
  }
  braked = false; md.setM1Speed(0);
  if (settle_next == S_SEEK_HOME && lim12()) settle_next = S_BACK_HOME;   // already home
  enterCal(settle_next);
}

void stepMotion(long e) {
  // a trip from the ISR (or found already open at the start of a move)
  if (braked) {
    uint8_t want = 0;
    if (cst == S_SEEK_HOME || cst == S_APPR_HOME) want = 12;
    if (cst == S_SEEK_END  || cst == S_APPR_END)  want = 11;
    if (hit_pin != want) { abortCal(hit_pin == 11 ? F("unexpected motor-end limit")
                                                   : F("unexpected home limit")); return; }
    if (cst == S_SEEK_HOME)      { seek_hit = hit_cnt; goSettle(S_BACK_HOME); }
    else if (cst == S_APPR_HOME) { home_raw = hit_cnt; goSettle(S_BACK_HOME2); }
    else if (cst == S_SEEK_END)  { seek_hit = hit_cnt; goSettle(S_BACK_END); }
    else                         { end_cnt = hit_cnt;  goSettle(S_RETURN); }
    return;
  }
  switch (cst) {
    case S_BACK_HOME:  stepBackoff(e, lim12, S_APPR_HOME); break;
    case S_BACK_HOME2: stepBackoff(e, lim12, S_SEEK_END);  break;
    case S_BACK_END:   stepBackoff(e, lim11, S_APPR_END);  break;
    case S_RETURN:
      if (!c_slow && e <= PARK_COUNTS + SLOWDOWN_COUNTS) { c_slow = true; c_target = -APPROACH_SPEED; }
      if (e <= PARK_COUNTS) goSettle(S_DONE);
      break;
    default: break;
  }
  if (cst == S_SETTLE) return;

  if (millis() - phase_t0 > PHASE_TIMEOUT_MS) { abortCal(F("phase timeout")); return; }
  if (labs(e - entry_e) > MAX_SEEK_COUNTS)   { abortCal(F("switch not found in max travel")); return; }
  if (move_dir * (e - entry_e) < -WRONG_WAY_COUNTS) { abortCal(F("moving the wrong way")); return; }
  if (millis() - win_t >= STALL_MS) {
    if (c_speed == c_target && c_boost_maxed() && labs(e - win_e) < STALL_COUNTS) { abortCal(F("stall at full boost")); return; }
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
  cal_ok = false;
  driver(true);
  delay(5);
  if (md.getM1Fault()) { abortCal(F("driver fault at start")); return; }
  noInterrupts(); idx_bad = 0; idx_n = 0; idx_seen = false; interrupts();
  Serial.println(F("C,=== calibration start ==="));
  motor_sign = 1;              // provisional: the probe runs in raw motor terms
  c_oc_run = 0; c_boost_peak = 0;
  dacc_f = dacc_b = 0; dn_f = dn_b = 0; drag_ok = false;
  startMove(S_PROBE, +1, PROBE_SPEED, false);   // bounded by count, not by switches
}

// One calibration tick: overcurrent, the state machine, then the ramp.
void stepCal(long e) {
  if (cst != S_SETTLE) {
    // overcurrent only counts while driving; a brake spike is expected
    if (!braked && mA_filt > MAX_CURRENT_MA) c_oc_run++; else c_oc_run = 0;
    if (c_oc_run >= 3) { abortCal(F("overcurrent")); return; }
  }
  if (hx_new) {
    hx_new = false;
    if (!braked && cst == S_SEEK_HOME) {
      float kg = (counts - force_zero) / COUNTS_PER_KG;
      if (kg < c_fmin) c_fmin = kg;
      else if (kg - c_fmin > HOME_F_ABORT_KG) { abortCal(F("pedal stop hit before D12 - lengthen the cell rod")); return; }
    }
    if (!braked && c_speed == c_target && e > 2048 && e < 17067) {
      if (cst == S_SEEK_END)                { dacc_f += counts; dn_f++; }
      else if (cst == S_RETURN && !c_slow)  { dacc_b += counts; dn_b++; }
    }
  }
  if      (cst == S_PROBE)  stepProbe(e);
  else if (cst == S_SETTLE) stepSettle(e);
  else                      stepMotion(e);

  static unsigned long last_ramp = 0;
  unsigned long dt = millis() - last_ramp;
  if (dt) {
    last_ramp = millis();
    if (!braked && calRunning() && cst != S_SETTLE) {
      int step = max(1, (int)(400UL * dt / RAMP_MS));
      if      (c_speed < c_target - step) c_speed += step;
      else if (c_speed > c_target + step) c_speed -= step;
      else                                c_speed  = c_target;
      int out = c_speed;
      if (cst != S_PROBE && c_speed == c_target && c_target != 0) {
        float vt = abs(c_target) * 0.127f;   // free-running counts/ms: 70 ran 8.9 on 2026-11-02
        float va = move_dir > 0 ? vel_cpms : -vel_cpms;           // along the move
        if (va < 0.6f * vt) c_boost += dt; else if (va > 0.9f * vt && c_boost > 0) c_boost -= dt;
        int cap = (int)(I_CAP_PER_A * CAL_I_CAP_A + I_CAP_OFFSET + I_CAP_PER_V * (va > 0 ? va : 0));
        if (c_boost < 0) c_boost = 0;
        if (abs(c_target) + c_boost > cap) c_boost = max(0, cap - abs(c_target));
        out = c_target + move_dir * c_boost;
        if (c_boost > c_boost_peak) { c_boost_peak = c_boost; c_boost_at = e; }
      }
      md.setM1Speed(pwm12(out) * motor_sign);
    }
  }
  speed_now = c_speed;   // for the telemetry line
}
