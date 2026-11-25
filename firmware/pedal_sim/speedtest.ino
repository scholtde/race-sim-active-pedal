/*
 * Braking-distance test:  v<speed>  (disarmed, calibrated).
 *
 * Raising SPEED_MAX is only safe once the stopping distance at the new speed
 * is known: on a fault the rail can reach a switch at full speed, and the
 * levers only have 1.38 mm (home) / 1.48 mm (end) of headroom past the trip.
 * So measure it where nothing can be hit -- mid-travel:
 *   1. from park, ramp toward the motor end, BRAKE at 60% of travel
 *   2. from there, ramp back toward home, BRAKE at 40% of travel
 *   3. return to park
 * Each brake is the same register brake the limit ISR uses. Reports the speed
 * actually reached, the braking distance, peak current and any bad index
 * checks (x2 decoding at that speed).
 *
 * Blocking, but it polls 'x' and keeps the limit ISR armed (guard) throughout.
 */
const unsigned long VT_RUN_TIMEOUT_MS = 4000;

// Run one leg at speed s (signed, encoder direction) until pos crosses brake_at,
// then brake. Returns false (and has braked/disabled) on any abort.
bool vtLeg(int s, long brake_at, const __FlashStringHelper *name) {
  int8_t dir = s > 0 ? 1 : -1;
  // stop well short of the soft limit whatever happens
  long fence = dir > 0 ? soft_max - TAPER_COUNTS / 2 : soft_min + TAPER_COUNTS / 2;   // 2.5 mm
  guard = dir; braked = false;
  int sp = 0, peak_mA = 0;
  unsigned long t0 = millis(), last_ramp = millis();
  long v_e = encRead(); unsigned long v_t = micros(); float v = 0;
  noInterrupts(); unsigned long bad0 = idx_bad, zc0 = z_corr_n; interrupts();
  while (true) {
    if (Serial.available() && Serial.read() == 'x') { brakeNow(); delay(150); Serial.println(F("V,ABORT,user")); return false; }
    long e = encRead();
    if (braked) { Serial.println(F("V,ABORT,limit switch hit")); return false; }   // ISR braked
    if ((dir > 0 && e >= fence) || (dir < 0 && e <= fence)) { brakeNow(); Serial.println(F("V,ABORT,reached fence")); return false; }
    if (millis() - t0 > VT_RUN_TIMEOUT_MS) { brakeNow(); Serial.println(F("V,ABORT,timeout")); return false; }
    int mA = md.getM1CurrentMilliamps(); if (mA > peak_mA) peak_mA = mA;
    if (md.getM1Fault()) { brakeNow(); Serial.println(F("V,ABORT,driver fault")); return false; }
    // speed over the last ~20 ms
    unsigned long now_us = micros();
    if (now_us - v_t >= 20000UL) { v = (e - v_e) * 1e6f / (float)(now_us - v_t); v_e = e; v_t = now_us; }
    if ((dir > 0 && e >= brake_at) || (dir < 0 && e <= brake_at)) {
      brakeNow(); braked = true;
      long e_brake = e; float v_brake = v;
      // wait until still for 60 ms
      long last = e; unsigned long still = millis(), tb = millis();
      while (millis() - still < 60 && millis() - tb < 2000) {
        long e2 = encRead(); if (e2 != last) { last = e2; still = millis(); }
      }
      long dist = labs(last - e_brake);
      noInterrupts(); unsigned long bad = idx_bad - bad0, zc = z_corr_n - zc0; int16_t zm = z_corr_max; interrupts();
      Serial.print(F("V,")); Serial.print(name);
      Serial.print(F(",cmd=")); Serial.print(s);
      Serial.print(F(",v=")); Serial.print(fabs(v_brake) / COUNTS_PER_MM, 1);
      Serial.print(F("mm/s,brake=")); Serial.print(dist / COUNTS_PER_MM, 2);
      Serial.print(F("mm,peak_mA=")); Serial.print(peak_mA);
      Serial.print(F(",idx_bad=")); Serial.print(bad);
      Serial.print(F(",zcorr=")); Serial.print(zc); Serial.print(F(",zmax=")); Serial.println(zm);
      braked = false; md.setM1Speed(0);
      return true;
    }
    unsigned long dt = millis() - last_ramp;
    if (dt) {
      last_ramp = millis();
      int step = max(1, (int)(400UL * dt / 400UL));   // same 400 ms full-scale ramp
      if (abs(sp) < abs(s)) sp += dir * step;
      if (abs(sp) > abs(s)) sp = s;
      md.setM1Speed(sp * motor_sign);
    }
  }
}

void speedTest(int s) {
  if (!cal_ok) { Serial.println(F("V,ABORT,not calibrated")); return; }
  s = constrain(abs(s), SPEED_MIN, 400);
  disarm(NULL); driver(true); delay(5);
  Serial.print(F("V,=== braking test at speed ")); Serial.print(s); Serial.println(F(" ==="));
  bool ok = vtLeg(+s, travel * 6 / 10, F("toward_end"))
         && vtLeg(-s, travel * 4 / 10, F("toward_home"));
  // back to park at the proven speed, then brake
  if (ok) {
    guard = -1; braked = false;
    unsigned long t0 = millis();
    while (millis() - t0 < 5000) {
      long e = encRead();
      if (braked || e <= PARK_COUNTS) break;
      md.setM1Speed((e - PARK_COUNTS > 1500 ? -RETURN_SPEED : -SPEED_MIN) * motor_sign);
    }
    brakeNow(); delay(100);
  }
  Serial.println(F("V,=== done ==="));
  braked = false; driver(false);
}
