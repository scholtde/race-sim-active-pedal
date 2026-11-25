/*
 * Active pedal -- HAND PROBE (no motor, driver held disabled the whole time).
 * Encoder A=D3 (INT1 x2) B=D8 Z=D10; limits D11 (motor end) / D12 (encoder end),
 * NC to GND with pull-ups; HX711 DOUT=D7 SCK=D5 (port I/O, interrupts on).
 * Prints:  L,<count>,d11=..,d12=..   at every limit edge (latched in the ISR)
 *          P,<count>,<mm>,idx_n,idx_bad,d11,d12,hx_raw,hx_min,hx_max,a2,a2min,a2max   every 100 ms
 *          (A2 = throttle pot, added 2026-11-15)
 * Command: z = zero count + clear stats
 */
#include <DualVNH5019MotorShield.h>
DualVNH5019MotorShield md;
volatile long enc = 0, idx_last = 0;
volatile unsigned long idx_n = 0, idx_bad = 0;
volatile bool idx_seen = false;
volatile uint8_t pb_prev = 0, a_last = 0;
struct LimEv { long cnt; uint8_t pb; };
volatile LimEv lim_ev[16]; volatile uint8_t lim_w = 0; uint8_t lim_r = 0;

ISR(INT1_vect) {
  uint8_t a = PIND & _BV(3); if (a == a_last) return; a_last = a;
  bool b = PINB & _BV(0);
  if ((a != 0) != b) enc++; else enc--;
}
ISR(PCINT0_vect) {
  uint8_t pb = PINB, rose = pb & ~pb_prev, chg = pb ^ pb_prev; pb_prev = pb;
  if (chg & (_BV(3) | _BV(4))) { lim_ev[lim_w & 15].cnt = enc; lim_ev[lim_w & 15].pb = pb; lim_w++; }
  if (rose & _BV(2)) {
    long e = enc;
    if (idx_seen) { long d = labs(e - idx_last); if (labs(d - 2048) > 2 && d > 3) idx_bad++; }
    idx_seen = true; idx_last = e; idx_n++;
  }
}
long encRead() { noInterrupts(); long e = enc; interrupts(); return e; }
long hxRead() {
  unsigned long v = 0;
  for (uint8_t i = 0; i < 24; i++) {
    PORTD |= _BV(5); delayMicroseconds(1);
    v = (v << 1) | ((PIND & _BV(7)) ? 1UL : 0UL);
    PORTD &= ~_BV(5); delayMicroseconds(1);
  }
  PORTD |= _BV(5); delayMicroseconds(1); PORTD &= ~_BV(5); delayMicroseconds(1);
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (long)v;
}
void setup() {
  md.init(); md.setM1Speed(0);
  digitalWrite(6, LOW); pinMode(6, OUTPUT);          // bridge OFF, never enabled here
  TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0));
  pinMode(7, INPUT); pinMode(8, INPUT); pinMode(10, INPUT); pinMode(3, INPUT);
  pinMode(11, INPUT_PULLUP); pinMode(12, INPUT_PULLUP);
  pinMode(5, OUTPUT); digitalWrite(5, LOW);
  pb_prev = PINB; a_last = PIND & _BV(3);
  EICRA = (EICRA & ~(_BV(ISC11) | _BV(ISC10))) | _BV(ISC10);
  EIFR = _BV(INTF1); EIMSK |= _BV(INT1);
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT4); PCIFR = _BV(PCIF0); PCICR |= _BV(PCIE0);
  Serial.begin(115200);
  Serial.println(F("\n=== hand probe: driver disabled. z = zero ==="));
}
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { noInterrupts(); enc = 0; idx_seen = false; idx_n = idx_bad = 0; interrupts(); Serial.println(F("zeroed")); }
  }
  static long hx = 0, hmin = 0x7FFFFFFF, hmax = -0x7FFFFFFF; static unsigned long hx_n = 0, last_hx = 0;
  if (!(PIND & _BV(7))) { hx = hxRead(); hx_n++; last_hx = millis(); if (hx < hmin) hmin = hx; if (hx > hmax) hmax = hx; }
  while (lim_r != lim_w) {
    noInterrupts(); long c = lim_ev[lim_r & 15].cnt; uint8_t pb = lim_ev[lim_r & 15].pb; interrupts(); lim_r++;
    Serial.print(F("L,")); Serial.print(c);
    Serial.print(F(",d11=")); Serial.print((pb & _BV(3)) ? F("OPEN") : F("ok"));
    Serial.print(F(",d12=")); Serial.println((pb & _BV(4)) ? F("OPEN") : F("ok"));
  }
  static int a2 = 0, amin = 1023, amax = 0;
  a2 = analogRead(A2); if (a2 < amin) amin = a2; if (a2 > amax) amax = a2;
  static unsigned long last_tx = 0;
  if (millis() - last_tx >= 100) {
    last_tx = millis();
    long e = encRead(); noInterrupts(); unsigned long in = idx_n, ib = idx_bad; interrupts();
    Serial.print(F("P,")); Serial.print(e); Serial.print(','); Serial.print(e * 3.0 / 2048.0, 2);
    Serial.print(','); Serial.print(in); Serial.print(','); Serial.print(ib);
    Serial.print(','); Serial.print((PINB & _BV(3)) ? F("OPEN") : F("ok"));
    Serial.print(','); Serial.print((PINB & _BV(4)) ? F("OPEN") : F("ok"));
    Serial.print(','); Serial.print(millis() - last_hx > 500 ? F("HX_SILENT") : F(""));
    Serial.print(hx); Serial.print(','); Serial.print(hmin); Serial.print(','); Serial.print(hmax);
    Serial.print(','); Serial.print(a2); Serial.print(','); Serial.print(amin); Serial.print(','); Serial.println(amax);
    amin = 1023; amax = 0;
    hmin = 0x7FFFFFFF; hmax = -0x7FFFFFFF;
  }
}
