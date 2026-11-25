/*
 * Active pedal -- BRING-UP / SENSE ONLY.
 *
 * The motor is forced OFF and never driven by this sketch. Safe to run with the
 * VNH5019 shield unpowered (HX711 + Uno both run from USB 5V).
 *
 * Wiring:   HX711 DOUT = D3, SCK = D5.   Bühler motor on M1 (not driven here).
 * Upload is via the stock bootloader, so the board stays usable from the IDE.
 *
 * Serial 115200. Commands:  t = tare   s = toggle raw sample stream   ? = help
 */

// ---- Pololu Dual VNH5019 shield pin map -----------------------------------
const uint8_t M1INA = 2, M1INB = 4, M1EN = 6,  M1PWM = 9,  M1CS = A0;
const uint8_t M2INA = 7, M2INB = 8, M2EN = 12, M2PWM = 10, M2CS = A1;

// ---- HX711 ----------------------------------------------------------------
const uint8_t HX_DOUT = 3, HX_SCK = 5;

long  tare_offset = 0;
bool  stream_raw  = false;

void motorsHardOff() {
  // INA=INB=LOW and PWM=LOW -> outputs off regardless of the EN/DIAG pull-up.
  pinMode(M1INA, OUTPUT); digitalWrite(M1INA, LOW);
  pinMode(M1INB, OUTPUT); digitalWrite(M1INB, LOW);
  pinMode(M1PWM, OUTPUT); digitalWrite(M1PWM, LOW);
  pinMode(M2INA, OUTPUT); digitalWrite(M2INA, LOW);
  pinMode(M2INB, OUTPUT); digitalWrite(M2INB, LOW);
  pinMode(M2PWM, OUTPUT); digitalWrite(M2PWM, LOW);
  pinMode(M1EN, INPUT);   // EN/DIAG is bidirectional; leave it alone
  pinMode(M2EN, INPUT);
}

// Wait for the HX711 to assert data-ready (DOUT low). Returns false on timeout,
// so a missing/miswired HX711 can never hang setup() the way the old sketch did.
bool hxWaitReady(unsigned long timeout_ms) {
  unsigned long t0 = millis();
  while (digitalRead(HX_DOUT) == HIGH) {
    if (millis() - t0 > timeout_ms) return false;
  }
  return true;
}

// One 24-bit conversion, gain 128 / channel A (25 clock pulses).
long hxRead() {
  unsigned long v = 0;
  noInterrupts();
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(HX_SCK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | (digitalRead(HX_DOUT) ? 1UL : 0UL);
    digitalWrite(HX_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(HX_SCK, HIGH); delayMicroseconds(1);   // 25th pulse
  digitalWrite(HX_SCK, LOW);  delayMicroseconds(1);
  interrupts();
  if (v & 0x800000UL) v |= 0xFF000000UL;             // sign-extend 24 -> 32
  return (long)v;
}

// Measure the real conversion rate -> tells us how the RATE pin is strapped
// (10 SPS = tied low, 80 SPS = tied high). No datasheet needed.
float measureRate() {
  hxWaitReady(500); hxRead();
  unsigned long t0 = millis();
  const uint8_t N = 12;
  for (uint8_t i = 0; i < N; i++) {
    if (!hxWaitReady(500)) return -1.0;
    hxRead();
  }
  unsigned long dt = millis() - t0;
  return dt ? (1000.0 * N) / dt : -1.0;
}

void setup() {
  motorsHardOff();
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== active pedal bring-up (SENSE ONLY, motor held off) ==="));

  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, LOW);     // must stay low, >60us high powers the HX711 down
  pinMode(HX_DOUT, INPUT);
  delay(500);                    // HX711 settling after power-up

  if (!hxWaitReady(1000)) {
    Serial.println(F("HX711: NO DATA-READY on D3 -- check wiring/power (DOUT stuck high)"));
  } else {
    float r = measureRate();
    Serial.print(F("HX711: OK.  measured rate = "));
    Serial.print(r, 1);
    Serial.print(F(" SPS  -> RATE pin strapped "));
    Serial.println(r > 40.0 ? F("HIGH (80 SPS)") : F("LOW (10 SPS)"));
    // No auto-tare: report ABSOLUTE counts so separate captures stay comparable
    // (opening the serial port resets the Uno, which would otherwise zero out any
    // force being held at that moment).
    tare_offset = 0;
    Serial.println(F("tare offset = 0 (absolute counts)"));
  }
  Serial.println(F("cmds: t=tare  s=stream  ?=help"));
  Serial.println(F("stats: n, mean, min, max, p2p (counts, tare-removed), M1CS"));
}

void loop() {
  static long   mn = 2147483647L, mx = -2147483648L;
  static double sum = 0;
  static unsigned long n = 0, last = 0;

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      long acc = 0;
      for (uint8_t i = 0; i < 16; i++) { hxWaitReady(500); acc += hxRead() / 16; }
      tare_offset = acc;
      Serial.print(F("# tared, offset = ")); Serial.println(tare_offset);
    } else if (c == 's') {
      stream_raw = !stream_raw;
      Serial.print(F("# raw stream ")); Serial.println(stream_raw ? F("ON") : F("OFF"));
    } else if (c == '?') {
      Serial.println(F("# t=tare  s=toggle raw stream"));
    }
  }

  if (!hxWaitReady(300)) {
    Serial.println(F("# HX711 timeout"));
    return;
  }
  long raw = hxRead() - tare_offset;

  if (stream_raw) { Serial.print(F("R,")); Serial.println(raw); }

  sum += raw; n++;
  if (raw < mn) mn = raw;
  if (raw > mx) mx = raw;

  if (millis() - last >= 1000) {
    last = millis();
    Serial.print(F("S,n=")); Serial.print(n);
    Serial.print(F(",mean=")); Serial.print((long)(sum / (n ? n : 1)));
    Serial.print(F(",min=")); Serial.print(mn);
    Serial.print(F(",max=")); Serial.print(mx);
    Serial.print(F(",p2p=")); Serial.print(mx - mn);
    Serial.print(F(",M1CS=")); Serial.println(analogRead(M1CS));
    sum = 0; n = 0; mn = 2147483647L; mx = -2147483648L;
  }
}
