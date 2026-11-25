/*
 * Motor characterisation:  M  (disarmed, calibrated).
 *
 * The back-EMF current cap needs real numbers: winding resistance R, the
 * back-EMF constant Ke, and what the VNH5019 current-sense reading actually
 * is. Its CS pin follows HIGH-SIDE current only, so after the shield's
 * filtering it may read motor current or roughly supply (duty x motor) current.
 *
 *  A. standstill pulses, 12 ms, at duty 10/20/30/40 %, alternating direction:
 *     motor current at stall scales with duty, supply current with duty^2,
 *     so the reading ratio says which one CS is -- and gives R.
 *  B. steady speeds 100..400: ramp from park toward the motor end, settle,
 *     average current and shaft speed over 40 ms, brake, return to park.
 *     V*duty = Ke*v + I*R  ->  Ke.
 * Prints raw M lines; the fit is done off-board.
 *
 * Blocking. Polls 'x'. The limit ISR stays armed through the guard, and every
 * leg stops at a fence 4 mm short of the soft limit.
 */
const long MT_FENCE = 2731;   // 4 mm

static bool mtAbort() {
  if (Serial.available() && Serial.read() == 'x') { brakeNow(); Serial.println(F("M,ABORT,user")); return true; }
  if (braked) { Serial.println(F("M,ABORT,limit switch")); return true; }
  if (md.getM1Fault()) { brakeNow(); Serial.println(F("M,ABORT,driver fault")); return true; }
  return false;
}

static void mtSettle() {
  long last = encRead(); unsigned long still = millis(), t0 = millis();
  while (millis() - still < 60 && millis() - t0 < 2000) {
    long e = encRead(); if (e != last) { last = e; still = millis(); }
  }
}

static bool mtPark() {
  guard = -1; braked = false;
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    if (mtAbort()) return false;
    long e = encRead();
    if (e <= PARK_COUNTS) break;
    md.setM1Speed((e - PARK_COUNTS > 1500 ? -150 : -SPEED_MIN) * motor_sign);
  }
  brakeNow(); mtSettle(); md.setM1Speed(0);
  return true;
}

void motorTest() {
  if (!cal_ok) { Serial.println(F("M,ABORT,not calibrated")); return; }
  disarm(NULL); driver(true); delay(5);
  Serial.println(F("M,=== motor characterisation ==="));
  if (!mtPark()) { driver(false); return; }

  // ---- A: standstill pulses ----
  Serial.println(F("M,stall,cmd,duty,mA_avg,mA_peak,moved"));
  const int stall_cmd[] = { 40, 80, 120, 160 };
  for (uint8_t i = 0; i < 4; i++) {
    for (int8_t dir = 1; dir >= -1; dir -= 2) {
      if (mtAbort()) { driver(false); return; }
      guard = dir; braked = false;
      long e0 = encRead();
      unsigned long t0 = micros();
      long acc = 0; int n = 0, pk = 0;
      md.setM1Speed(dir * stall_cmd[i] * motor_sign);
      while (micros() - t0 < 12000UL) {
        if (micros() - t0 > 7000UL) {            // last 5 ms: past the L/R rise
          int mA = md.getM1CurrentMilliamps(); acc += mA; n++; if (mA > pk) pk = mA;
        }
      }
      long moved = encRead() - e0;
      brakeNow(); delay(250); md.setM1Speed(0);
      Serial.print(F("M,stall,")); Serial.print(dir * stall_cmd[i]);
      Serial.print(','); Serial.print(stall_cmd[i] / 4.0f, 1);
      Serial.print(','); Serial.print(n ? acc / n : 0);
      Serial.print(','); Serial.print(pk);
      Serial.print(','); Serial.println(moved);
    }
  }

  // ---- B: steady speeds ----
  Serial.println(F("M,steady,cmd,v_cpms,mA_avg,mA_min,mA_max,end_mm"));
  for (int cmd = 100; cmd <= 400; cmd += 50) {
    if (!mtPark()) { driver(false); return; }
    delay(150);
    guard = 1; braked = false;
    int sp = 0; unsigned long last = millis(), t_at = 0;
    bool ok = true, measured = false;
    while (true) {
      if (mtAbort()) { ok = false; break; }
      long e = encRead();
      if (e >= soft_max - MT_FENCE) { brakeNow(); Serial.print(F("M,fence before settling at cmd ")); Serial.println(cmd); break; }
      unsigned long dt = millis() - last;
      if (dt) { last = millis();
        if (sp < cmd) { sp += 2 * dt; if (sp > cmd) sp = cmd; md.setM1Speed(sp * motor_sign); } }
      if (sp == cmd && !t_at) t_at = millis();
      if (t_at && millis() - t_at >= 80) {
        // measure for 40 ms
        long e1 = encRead(); unsigned long u1 = micros();
        long acc = 0; int n = 0, mn = 32767, mx = 0;
        while (micros() - u1 < 40000UL) {
          int mA = md.getM1CurrentMilliamps(); acc += mA; n++;
          if (mA < mn) mn = mA; if (mA > mx) mx = mA;
        }
        long e2 = encRead(); unsigned long u2 = micros();
        brakeNow();
        float v = (e2 - e1) * 1000.0f / (float)(u2 - u1);
        Serial.print(F("M,steady,")); Serial.print(cmd);
        Serial.print(','); Serial.print(v, 2);
        Serial.print(','); Serial.print(acc / n);
        Serial.print(','); Serial.print(mn);
        Serial.print(','); Serial.print(mx);
        Serial.print(','); Serial.println(e2 / COUNTS_PER_MM, 1);
        measured = true;
        break;
      }
    }
    mtSettle(); md.setM1Speed(0);
    if (!ok) { driver(false); return; }
    (void)measured;
  }
  mtPark();
  Serial.println(F("M,=== done ==="));
  braked = false; driver(false);
}
