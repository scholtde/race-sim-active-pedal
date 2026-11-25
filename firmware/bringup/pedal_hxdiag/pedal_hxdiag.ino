/*
 * HX711 channel finder -- SENSE ONLY, motor forced off, shield may stay unpowered.
 *
 * Channel/gain is chosen by clock pulses after the 24 data bits:
 *   25 -> ch A gain 128 | 26 -> ch B gain 32 | 27 -> ch A gain 64
 * The count configures the NEXT conversion, and at 10 SPS the datasheet settling
 * time after a channel change is 400 ms -- so discard several conversions and
 * allow a long timeout, or every switch looks like a dead chip.
 *
 * HX711 DOUT = D3, SCK = D5.  Serial 115200.
 */
const uint8_t M1INA=2, M1INB=4, M1EN=6, M1PWM=9;
const uint8_t M2INA=7, M2INB=8, M2EN=12, M2PWM=10;
const uint8_t HX_DOUT = 3, HX_SCK = 5;

void motorsHardOff() {
  pinMode(M1INA,OUTPUT); digitalWrite(M1INA,LOW);
  pinMode(M1INB,OUTPUT); digitalWrite(M1INB,LOW);
  pinMode(M1PWM,OUTPUT); digitalWrite(M1PWM,LOW);
  pinMode(M2INA,OUTPUT); digitalWrite(M2INA,LOW);
  pinMode(M2INB,OUTPUT); digitalWrite(M2INB,LOW);
  pinMode(M2PWM,OUTPUT); digitalWrite(M2PWM,LOW);
  pinMode(M1EN,INPUT); pinMode(M2EN,INPUT);
}

// >60us high powers the chip down; dropping low wakes it and resets the channel
// selection back to A/128.
void hxReset() {
  digitalWrite(HX_SCK,HIGH); delayMicroseconds(100);
  digitalWrite(HX_SCK,LOW);  delay(50);
}

bool hxWaitReady(unsigned long ms) {
  unsigned long t0 = millis();
  while (digitalRead(HX_DOUT) == HIGH) if (millis()-t0 > ms) return false;
  return true;
}

long hxReadPulses(uint8_t pulses) {
  unsigned long v = 0;
  noInterrupts();
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(HX_SCK,HIGH); delayMicroseconds(1);
    v = (v << 1) | (digitalRead(HX_DOUT) ? 1UL : 0UL);
    digitalWrite(HX_SCK,LOW);  delayMicroseconds(1);
  }
  for (uint8_t i = 24; i < pulses; i++) {
    digitalWrite(HX_SCK,HIGH); delayMicroseconds(1);
    digitalWrite(HX_SCK,LOW);  delayMicroseconds(1);
  }
  interrupts();
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (long)v;
}

const long TIMEOUT = 0x7FFFFFFFL;

// Select a channel, throw away the settling conversions, then sample it N times.
bool sampleChannel(uint8_t pulses, uint8_t n, long &mn, long &mx, long &last) {
  for (uint8_t i = 0; i < 4; i++) {           // settle: >=400 ms at 10 SPS
    if (!hxWaitReady(1500)) return false;
    hxReadPulses(pulses);
  }
  mn = 2147483647L; mx = -2147483648L;
  for (uint8_t i = 0; i < n; i++) {
    if (!hxWaitReady(1500)) return false;
    last = hxReadPulses(pulses);
    if (last < mn) mn = last;
    if (last > mx) mx = last;
  }
  return true;
}

void report(const __FlashStringHelper *name, uint8_t pulses) {
  long mn, mx, last;
  Serial.print(name); Serial.print(F(": "));
  if (!sampleChannel(pulses, 6, mn, mx, last)) { Serial.println(F("TIMEOUT")); return; }
  Serial.print(F("val=")); Serial.print(last);
  Serial.print(F(" min=")); Serial.print(mn);
  Serial.print(F(" max=")); Serial.print(mx);
  Serial.print(F(" span=")); Serial.print(mx - mn);
  if (mx >= 8388000L && mn >= 8388000L)       Serial.println(F("   RAILED+ (input floating?)"));
  else if (mx <= -8388000L && mn <= -8388000L) Serial.println(F("   RAILED-"));
  else                                         Serial.println(F("   *** LIVE ***"));
}

void setup() {
  motorsHardOff();
  Serial.begin(115200);
  delay(300);
  pinMode(HX_SCK,OUTPUT); digitalWrite(HX_SCK,LOW);
  pinMode(HX_DOUT,INPUT);
  delay(500);
  Serial.println();
  Serial.println(F("=== HX711 channel finder v2 (motor off) ==="));
  Serial.println(F(">>> PRESS AND RELEASE THE LOAD CELL THROUGHOUT THIS CAPTURE <<<"));
}

void loop() {
  hxReset();
  report(F("A/128"), 25);
  report(F("B/32 "), 26);
  report(F("A/64 "), 27);
  Serial.println(F("---"));
}
