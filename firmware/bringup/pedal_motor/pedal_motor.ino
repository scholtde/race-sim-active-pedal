/*
 * Active pedal -- MOTOR BRING-UP.
 *
 * Motor is OFF at boot and can only ever move for a bounded, ramped pulse that
 * the firmware itself ends. There is no "run forever" mode: if the host stops
 * talking, or crashes, or USB drops, the pulse still expires on its own.
 *
 * Hardware: Pololu Dual VNH5019, Buhler motor on M1.  HX711 DOUT=D3 SCK=D5.
 * Load cell in line with the pedal -> pressing is COMPRESSION -> positive kg.
 *
 * Serial 115200. Commands (newline terminated):
 *    f <speed> <ms>   forward pulse   (speed 1..MAX_SPEED, ms 1..MAX_PULSE_MS)
 *    r <speed> <ms>   reverse pulse
 *    x                emergency stop
 *    c                read current + force with the motor off (baseline)
 *    t                tare the force zero
 *    ?                help / limits
 */
#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;          // default shield pins; motor on M1

// ---- HX711 (free pins; the shield uses D2,D4,D6,D7,D8,D9,D10,D12,A0,A1) ----
const uint8_t HX_DOUT = 3, HX_SCK = 5;

// ---- calibration (3-point least-squares fit, 0 / 1.5 / 7.5 kg) -------------
const float COUNTS_PER_KG = 46678.0;   // compression (pedal press) = positive
long  force_zero = 76414;

// ---- safety limits ---------------------------------------------------------
const float         MAX_FORCE_KG   = 40.0;    // cell is rated 100 kg / 200 kg peak
const int           MAX_CURRENT_MA = 3000;    // 12V 5A supply is the real limit
const int           MAX_SPEED      = 400;     // full scale; inrush is bounded by the ramp
const unsigned long MAX_PULSE_MS   = 2000;
const unsigned long RAMP_MS        = 400;     // longer soft-start: inrush, not load, was the limit

// ---- state -----------------------------------------------------------------
bool          running    = false;
int           cmd_speed  = 0;                 // signed target, -MAX..+MAX
unsigned long pulse_t0   = 0, pulse_len = 0;
long          counts     = 0;
int           last_mA    = 0;
char          line[24];
uint8_t       line_n     = 0;

// ---------------------------------------------------------------- HX711 -----
long hxRead() {
  unsigned long v = 0;
  noInterrupts();
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(HX_SCK,HIGH); delayMicroseconds(1);
    v = (v << 1) | (digitalRead(HX_DOUT) ? 1UL : 0UL);
    digitalWrite(HX_SCK,LOW);  delayMicroseconds(1);
  }
  digitalWrite(HX_SCK,HIGH); delayMicroseconds(1);      // 25th pulse: ch A, gain 128
  digitalWrite(HX_SCK,LOW);  delayMicroseconds(1);
  interrupts();
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (long)v;
}
// Non-blocking: never stall the control loop waiting on a 10 SPS converter.
inline void hxPoll() { if (digitalRead(HX_DOUT) == LOW) counts = hxRead(); }
inline float forceKg() { return (counts - force_zero) / COUNTS_PER_KG; }

// ---------------------------------------------------------------- motor -----
void motorOff() { md.setM1Speed(0); running = false; cmd_speed = 0; }

void abortPulse(const __FlashStringHelper *why) {
  motorOff();
  Serial.print(F("ABORT: ")); Serial.println(why);
}

void startPulse(int speed, unsigned long ms) {
  if (speed >  MAX_SPEED) speed =  MAX_SPEED;
  if (speed < -MAX_SPEED) speed = -MAX_SPEED;
  if (ms > MAX_PULSE_MS)  ms = MAX_PULSE_MS;
  if (ms == 0 || speed == 0) { Serial.println(F("bad args")); return; }
  if (md.getM1Fault()) { Serial.println(F("M1 FAULT - is the shield powered?")); return; }
  cmd_speed = speed; pulse_len = ms; pulse_t0 = millis(); running = true;
  Serial.print(F("PULSE speed=")); Serial.print(speed);
  Serial.print(F(" ms=")); Serial.println(ms);
}

// ---------------------------------------------------------------- cmds ------
void help() {
  Serial.println(F("f <speed> <ms> | r <speed> <ms> | x | c | t | ?"));
  Serial.print(F("limits: speed<=")); Serial.print(MAX_SPEED);
  Serial.print(F("/400  pulse<=")); Serial.print(MAX_PULSE_MS);
  Serial.print(F("ms  I<=")); Serial.print(MAX_CURRENT_MA);
  Serial.print(F("mA  F<=")); Serial.print(MAX_FORCE_KG); Serial.println(F("kg"));
}

void handleLine() {
  char c = line[0];
  if (c == 'x') { motorOff(); Serial.println(F("STOP")); return; }
  if (c == '?') { help(); return; }
  if (c == 't') {
    long acc = 0;
    for (uint8_t i = 0; i < 8; i++) {
      unsigned long t0 = millis();
      while (digitalRead(HX_DOUT) == HIGH && millis()-t0 < 400) {}
      acc += hxRead() / 8;
    }
    force_zero = acc;
    Serial.print(F("tared, zero=")); Serial.println(force_zero);
    return;
  }
  if (c == 'c') {
    Serial.print(F("idle: I=")); Serial.print(md.getM1CurrentMilliamps());
    Serial.print(F("mA  F=")); Serial.print(forceKg(), 3);
    Serial.print(F("kg  counts=")); Serial.print(counts);
    Serial.print(F("  fault=")); Serial.println(md.getM1Fault() ? F("YES") : F("no"));
    return;
  }
  if (c == 'f' || c == 'r') {
    int sp = 0; unsigned long ms = 0;
    if (sscanf(line + 1, "%d %lu", &sp, &ms) != 2) { Serial.println(F("usage: f <speed> <ms>")); return; }
    startPulse(c == 'f' ? sp : -sp, ms);
    return;
  }
  Serial.println(F("? for help"));
}

// ---------------------------------------------------------------- setup -----
void setup() {
  md.init();
  md.setM1Speed(0);                    // motor off before anything else
  Serial.begin(115200);
  pinMode(HX_SCK, OUTPUT); digitalWrite(HX_SCK, LOW);
  pinMode(HX_DOUT, INPUT);
  delay(500);
  Serial.println();
  Serial.println(F("=== active pedal motor bring-up ==="));
  Serial.println(F("motor OFF. pulses are bounded + ramped + current/force limited."));
  help();
  Serial.println(F("telemetry: T,ms,speed,mA,kg"));
}

// ---------------------------------------------------------------- loop ------
void loop() {
  // --- serial ---
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line_n) { line[line_n] = 0; handleLine(); line_n = 0; }
    } else if (line_n < sizeof(line) - 1) {
      line[line_n++] = ch;
    }
  }

  hxPoll();
  last_mA = md.getM1CurrentMilliamps();

  if (running) {
    unsigned long el = millis() - pulse_t0;

    if (el >= pulse_len)                    { motorOff(); Serial.println(F("done")); }
    else if (md.getM1Fault())               { abortPulse(F("driver fault (M1)")); }
    else if (last_mA > MAX_CURRENT_MA)      { abortPulse(F("overcurrent")); }
    else if (fabs(forceKg()) > MAX_FORCE_KG){ abortPulse(F("force limit")); }
    else {
      // soft-start ramp, then hold
      long s = cmd_speed;
      if (el < RAMP_MS) s = (long)cmd_speed * (long)el / (long)RAMP_MS;
      md.setM1Speed((int)s);

      static unsigned long last_tx = 0;
      if (millis() - last_tx >= 50) {       // 20 Hz telemetry
        last_tx = millis();
        Serial.print(F("T,")); Serial.print(el);
        Serial.print(','); Serial.print((int)s);
        Serial.print(','); Serial.print(last_mA);
        Serial.print(','); Serial.println(forceKg(), 3);
      }
    }
  } else {
    static unsigned long last_idle = 0;      // 1 Hz heartbeat when idle
    if (millis() - last_idle >= 1000) {
      last_idle = millis();
      Serial.print(F("I,")); Serial.print(last_mA);
      Serial.print(','); Serial.print(forceKg(), 3);
      Serial.print(','); Serial.println(counts);
    }
  }
}
