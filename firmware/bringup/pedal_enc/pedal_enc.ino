/*
 * Active pedal -- ENCODER BRING-UP.
 *
 * Yumo E6B2-CWZ3E on the motor shaft -- sold as 1000 ppr, MEASURED 1024 ppr
 * (exactly 2048 counts between index pulses, every full rev). Voltage output (NPN with an
 * internal pull-up to its own supply), so it MUST be powered from 5 V, not 12.
 *   A (black)  -> D3   INT1, every edge  -> 2048 counts/rev (x2 decode)
 *   B (white)  -> D8   read inside the A interrupt
 *   Z (orange) -> D10  pin-change interrupt, once per rev
 * HX711 DOUT moved D3 -> D7 to free INT1.
 *
 * Limit switches, NC contacts, commons to GND, internal pull-ups:
 *   D11 = motor end   (CW / counts up / pedal pressed: end of travel)
 *   D12 = encoder end (CCW / counts down / pedal released: home)
 * Closed = LOW = ok. Tripped OR a broken wire = HIGH = stop. A signal wire
 * shorted to a grounded frame reads ok forever -- that failure is not caught.
 *
 * Every index pulse checks that exactly one rev went by since the last one.
 * Reversing across the index fires Z again ~0 counts later, which is fine;
 * anything else = missed or phantom edges (noise, or the ISR too slow).
 *
 * Boots with the M1 bridge DISABLED: EN/DIAG (D6) is held low, which turns all
 * four output FETs off. The motor can neither be driven nor braked (speed 0 in
 * the library is brake-to-GND, which makes the shaft stiff to turn by hand).
 *
 * Commands:  e = enable driver   d = disable   m<speed> = spin M1 (ramped,
 *            +-400, needs e)   x = stop   z = zero
 */
#include <DualVNH5019MotorShield.h>
DualVNH5019MotorShield md;

const long COUNTS_PER_REV = 2048;
const int  MAX_CURRENT_MA = 14000;
const unsigned long RAMP_MS = 400;   // full-scale ramp; less trips on inrush

volatile long enc = 0;
volatile long idx_last = 0;           // enc at the previous index
volatile long idx_delta = 0;          // counts between the last two indices
volatile unsigned long idx_n = 0, idx_bad = 0;
volatile bool idx_seen = false;
volatile uint8_t pb_prev = 0;
volatile uint8_t lim_trips = 0;        // bit0 = D11 motor end, bit1 = D12 encoder end
// Exact encoder count at each limit edge, latched in the ISR (a polled log is
// hundreds of counts late at hand speed). Small ring, drained by loop().
struct LimEv { long cnt; uint8_t pb; };
volatile LimEv lim_ev[8]; volatile uint8_t lim_w = 0; uint8_t lim_r = 0;
volatile long bad_last = 0, bad_at = 0;   // the most recent wrong index delta

// x2 decode: on every A edge, A==B one way, A!=B the other.
ISR(INT1_vect) {
  bool a = PIND & _BV(3), b = PINB & _BV(0);
  if (a != b) enc++; else enc--;
}

// One vector for port B: Z (D10 = PB2) and both limits (D11 = PB3, D12 = PB4).
ISR(PCINT0_vect) {
  uint8_t pb = PINB, rose = pb & ~pb_prev, chg = pb ^ pb_prev;
  pb_prev = pb;
  if (chg & (_BV(3) | _BV(4))) {
    lim_ev[lim_w & 7].cnt = enc; lim_ev[lim_w & 7].pb = pb; lim_w++;
  }
  if (pb & (_BV(3) | _BV(4))) {       // a limit is open: kill the bridge NOW,
    PORTD &= ~_BV(6); DDRD |= _BV(6); // not whenever loop() next gets round to it
    lim_trips |= ((pb >> 3) & 1) | (((pb >> 4) & 1) << 1);
  }
  if (!(rose & _BV(2))) return;       // Z rising edge only
  long e = enc;
  if (idx_seen) {
    idx_delta = e - idx_last;
    long d = labs(idx_delta);
    if (d != COUNTS_PER_REV && d > 3) { idx_bad++; bad_last = idx_delta; bad_at = e; }
  }
  idx_seen = true; idx_last = e; idx_n++;
}

long encRead() { noInterrupts(); long e = enc; interrupts(); return e; }

int target = 0, speed_now = 0;
bool drv_on = false;

// EN/DIAG is open-drain with a pull-up on the shield: drive it LOW to disable,
// release it (INPUT) to enable. Never drive it HIGH.
bool limitOpen() { return PINB & (_BV(3) | _BV(4)); }

void driver(bool on) {
  if (on && limitOpen()) {
    Serial.println(F("refused: a limit switch is open - turn off it by hand first"));
    return;
  }
  target = speed_now = 0; md.setM1Speed(0);
  if (on) pinMode(6, INPUT); else { digitalWrite(6, LOW); pinMode(6, OUTPUT); }
  drv_on = on;
  Serial.println(on ? F("driver ENABLED") : F("driver DISABLED (outputs off, shaft free)"));
}

void setup() {
  md.init(); md.setM1Speed(0);
  digitalWrite(6, LOW); pinMode(6, OUTPUT);   // bridge off before anything else
  // md.init() still claims the unused M2 pins as outputs and enables Timer1
  // PWM on D10 (OC1B). Take them back for the HX711, encoder B and Z.
  TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0));
  pinMode(7, INPUT); pinMode(8, INPUT); pinMode(10, INPUT); pinMode(3, INPUT);
  pinMode(11, INPUT_PULLUP); pinMode(12, INPUT_PULLUP);
  pb_prev = PINB;
  // The CWZ3E has its own 2k pull-ups; ours are harmless in parallel.
  EICRA = (EICRA & ~(_BV(ISC11) | _BV(ISC10))) | _BV(ISC10);   // INT1 on CHANGE
  EIFR = _BV(INTF1); EIMSK |= _BV(INT1);
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT4); PCIFR = _BV(PCIF0); PCICR |= _BV(PCIE0);
  Serial.begin(115200);
  Serial.println(F("\n=== encoder bring-up: m<speed> x z ? ==="));
  driver(false);
  Serial.println(F("E,counts,rev,rpm,idx_n,idx_delta,idx_bad,speed,mA,drv,lim_motor,lim_enc,trips"));
}

void loop() {
  static char line[12]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { line[n] = 0; n = 0;
        if (line[0] == 'e') driver(true);
        else if (line[0] == 'd') driver(false);
        else if (line[0] == 'm') {
          if (drv_on) target = constrain(atoi(line + 1), -400, 400);
          else Serial.println(F("driver disabled - send e first"));
        }
        else if (line[0] == 'x') { target = 0; speed_now = 0; md.setM1Speed(0); }
        else if (line[0] == 'z') { noInterrupts(); enc = 0; idx_seen = false;
                                   idx_n = idx_bad = 0; idx_delta = 0; lim_trips = 0; interrupts(); }
        else Serial.println(F("e d m<speed> x z"));
      }
    } else if (n < sizeof(line) - 1) line[n++] = c;
  }

  static unsigned long last_ramp = 0;
  unsigned long dt = millis() - last_ramp;
  if (dt) {
    last_ramp = millis();
    int step = max(1, (int)(400UL * dt / RAMP_MS));
    if      (speed_now < target - step) speed_now += step;
    else if (speed_now > target + step) speed_now -= step;
    else                                speed_now  = target;
    if (drv_on) md.setM1Speed(speed_now);
  }
  if (drv_on && !(PIND & _BV(6)) && limitOpen()) {   // the ISR pulled EN low
    drv_on = false; target = speed_now = 0; md.setM1Speed(0);
    Serial.println(F("STOP: limit switch - driver disabled"));
  }
  while (lim_r != lim_w) {
    noInterrupts(); LimEv ev = { lim_ev[lim_r & 7].cnt, lim_ev[lim_r & 7].pb }; interrupts();
    lim_r++;
    Serial.print(F("L,")); Serial.print(ev.cnt);
    Serial.print(F(",motor=")); Serial.print((ev.pb & _BV(3)) ? F("OPEN") : F("ok"));
    Serial.print(F(",enc="));   Serial.println((ev.pb & _BV(4)) ? F("OPEN") : F("ok"));
  }
  { static unsigned long seen_bad = 0;
    noInterrupts(); unsigned long nb = idx_bad; long bl = bad_last, ba = bad_at; interrupts();
    if (nb != seen_bad) { seen_bad = nb;
      Serial.print(F("BAD,index delta=")); Serial.print(bl);
      Serial.print(F(" at count=")); Serial.println(ba); } }
  int mA = md.getM1CurrentMilliamps();
  if (drv_on && (mA > MAX_CURRENT_MA || md.getM1Fault())) {   // D6 reads low while disabled
    target = speed_now = 0; md.setM1Speed(0);
    Serial.println(F("STOP: overcurrent/fault"));
  }

  static unsigned long last_tx = 0; static long last_e = 0;
  if (millis() - last_tx >= 100) {
    unsigned long now = millis(), el = now - last_tx; last_tx = now;
    long e = encRead();
    float rpm = (e - last_e) * 60000.0 / COUNTS_PER_REV / el; last_e = e;
    noInterrupts(); unsigned long in = idx_n, ib = idx_bad; long id = idx_delta; interrupts();
    Serial.print(F("E,")); Serial.print(e);
    Serial.print(','); Serial.print((float)e / COUNTS_PER_REV, 3);
    Serial.print(','); Serial.print(rpm, 0);
    Serial.print(','); Serial.print(in);
    Serial.print(','); Serial.print(id);
    Serial.print(','); Serial.print(ib);
    Serial.print(','); Serial.print(speed_now);
    Serial.print(','); Serial.print(mA);
    Serial.print(drv_on ? F(",on") : F(",off"));
    // limit state: 1 = closed/ok, 0 = OPEN (tripped or broken)
    Serial.print(','); Serial.print((PINB & _BV(3)) ? 0 : 1);
    Serial.print(','); Serial.print((PINB & _BV(4)) ? 0 : 1);
    Serial.print(','); Serial.println(lim_trips);
  }
}
