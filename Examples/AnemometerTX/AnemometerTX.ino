// AnemometerTX: read a Davis anemometer and transmit it as a Davis transmitter (the ISS SIM board).
//
// The packets follow the VP2 ISS type sequence and timing. Wind speed and direction go in every packet,
// the 10-minute gust in the gust packets, the other packet types carry Davis' "no sensor" values. The radio
// sleeps between transmissions.
//
// Wind speed: by default the mean over all rotations since the previous packet. Built with
// -DWIND_GATE_US=2250000, the rotations counted in a 2.25 s gate ending before each transmission, as the ISS
// does (1 rotation in 2.25 s is 1 mph). A rotation is the first edge after the switch has rested open for
// DEBOUNCE_US, so contact bounce on closing and on opening is ignored.
//
// Gust packet, as the ISS sends it: byte 3 the highest wind speed in the last 10 gust periods (a gust period is
// the 20-packet sequence), byte 5 upper nibble the age of that speed in gust periods (0..9), lower nibble its
// direction rounded to 16 sectors, (dir + 8) >> 4; byte 4 is 1 (5 = no anemometer).
//
// Transmit timing: a Davis receiver expects the interval (40 + ID) / 16 s. A board with a quartz crystal can run
// on its own clock (c0, with a fixed correction p). A Moteino's ceramic resonator is too far off and drifts
// with temperature, so with c1 the board measures its clock against the RFM69's crystal every CAL_PERIOD_S,
// using the RFM69's RX timeout (see DavisRFM69::timeRxTimeout()), and corrects the interval with that.
// A calibration is a burst of CAL_BURST measurements, each a long timeout in one transmit gap and a short one in
// the next; their difference cancels the RFM69's start-up delay. The median of the burst is applied.
//
// Hardware (Moteino or compatible ATmega328P with an RFM69):
//   anemometer reed switch between WIND_PIN (INT1) and GND
//   wind vane potentiometer: one end on POT_VCC, wiper on POT_PIN, other end on GND;
//   POT_BLIND_VCC through an extra resistor to the POT_VCC end, for detecting the dead zone
//
// Output: T lines for transmitted packets, as RXTX's:
//   T <channel> <10 bytes> <interval> <target> <timing ppm> <calibrations> <rejects> 0.0
// where interval is the transmit interval that just ended and target the one it was scheduled for (micros),
// where calibrations counts the calibration bursts and rejects the failed measurements, and with c1
//   K <long us> <short us> <measured ppm|fail>          for each measurement
//   C <median ppm|fail> <measurements> <timing ppm>     at the end of each calibration burst
//
// Commands (end with CR or LF; settings take effect at once, e saves them to EEPROM), as RXTX's:
//   m<0..8>     band: 0 US, 1 EU, 2 AU, 3 NZ, 4 JP, 5 IN, 6 R2, 7 RU, 8 TW (the Davis firmware's band numbers)
//   i<0..8>     transmitter ID to send as, 0 = transmitter off
//   c<0|1>      transmit timing: 0 fixed at p (quartz), 1 calibrated against the RFM69's crystal, plus p (resonator)
//   p<ppm>      transmit timing correction (ppm, integer); with c1, the RFM69 crystal's own error
//   h<0|1>      RFM69 module: 0 W/CW (PA0, +13 dBm), 1 HW/HCW (PA_BOOST, +20 dBm); a mismatch transmits only leakage
//   g<0|1>      on-air framing: 0 legacy (32 bit lead, 48 bit preamble), 1 as Davis (18 bit lead, 32 bit preamble, 10.4 kHz)
//   e           save settings to EEPROM
//   d           load the defaults (not saved)
//   r           reset
//   ?           show settings

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#include "DavisRFM69.h"

#define NAME_VERSION F("AnemometerTX v2026112401")
#define SERIAL_BAUD 115200
#define SERBUF_LEN 8
#define CFG_MAGIC 0x4154  // change when Config changes
#define LED 9             // Moteinos have the LED on D9; -1 for none
#define LED_TX_MS 100     // LED blink on each transmission

// ----- Wind sensor -----

#define WIND_PIN 3            // anemometer reed switch, INT1
#define POT_PIN A0            // vane potentiometer wiper
#define POT_VCC 5             // vane potentiometer supply, driven only while reading
#define POT_BLIND_VCC 4       // supply through the extra resistor, for dead zone detection
#define ANALOG_WAIT_MS 1      // settling time after switching the supply
#define ANALOG_ROUNDING 0     // on a Leonardo set it to 1..2, its pins don't source exactly Vcc
#define GAP_DETECT_THRESH1 5  // dead zone detection thresholds
#define GAP_DETECT_THRESH2 50
#define GAP_DETECT_THRESH3 70

#define DEBOUNCE_US 5000UL        // the switch must be open this long before a closure counts (contact bounce)
#define WIND_CALM_US 2250000UL    // no rotation for this long is calm (below 1 mph)
#define WIND_RPS_MPH 2.25         // 1 revolution per second is 2.25 mph (Davis)
#define WIND_LEAD_US 100000UL     // read the sensors this long before a transmission
// #define WIND_GATE_US 2250000UL // count rotations in a gate of this length instead of averaging periods
#define GUST_SLOTS 10             // gust periods in the gust window

// ----- Clock calibration (c1) -----

#ifndef CAL_PERIOD_S
#define CAL_PERIOD_S 1800     // seconds between calibration bursts; 0 = continuously (testing)
#endif
#ifndef CAL_BURST
#define CAL_BURST 4           // measurements per calibration, their median is applied; more for a noisier board
#endif
#define CAL_BITRATE 17256     // RFM69 bitrate register: bit time 539.25 us, timeout unit 8.628 ms
#define CAL_N_LONG 255        // 2.200140 s, fits in the shortest transmit gap (ID 1, 2.5625 s)
#define CAL_N_SHORT 8         // 0.069024 s; the difference, 2.131116 s, is the measured span
#define CAL_UNIT_PPM 1095     // the RFM69's timeout unit is this much longer than 16 bit times (measured on an
                              // RFM69HCW and an RFM69CW against NTP: 1095 +/- 5 ppm, at all bitrates)
#define CAL_MAX_PPM 1000      // a measurement further off is a failure, not the board's clock
#define CAL_MAX_FAILS 8       // a burst is given up after this many failed measurements

#define S_OK true
#define S_ERR false

struct Config {
  uint16_t magic;
  byte bandRaw;       // m
  byte txId;          // i, 1..8, 0 = off
  byte learn;         // c
  int16_t ppm;        // p
  byte highPower;     // h
  byte frame;         // g
};

Config cfg;

DavisRFM69 radio;
bool localEcho = false;

// anemometer pulses, written in the wind ISR
volatile uint32_t lastPulse;      // time of the last debounced pulse, micros
volatile uint32_t lastEdge;       // time of the last edge, bounces included, micros
volatile bool lineOpen = true;    // the switch's level after the last edge
#ifdef WIND_GATE_US
volatile uint32_t gateStart;      // the gate for the next reading, micros
volatile uint16_t gateCount;      // rotations in it
#endif
volatile uint32_t lastPeriod;     // the last rotation period, 0 if unknown
volatile uint32_t windowStart;    // the pulse the averaging window starts at
volatile uint16_t windowCount;    // rotations since windowStart
volatile bool pulseSeen = false;

// wind data for the next packet, written in loop(), read in the transmit ISR
volatile byte windSpeed = 0;      // mph
volatile byte windDir = 0;        // raw vane value, 0 in the dead zone
bool windRead = false;            // read for the coming transmission

// gust: the highest speed and its 16-sector direction in each gust period, slot gustCur is the running one
byte gustMax[GUST_SLOTS], gustDir[GUST_SLOTS];
byte gustCur = 0;
volatile byte gustSpeed = 0;      // byte 3 of the next gust packet
volatile byte gustInfo = 0;       // byte 5: age << 4 | direction

byte seqIndex = 0;

// transmit report, filled in the transmit ISR
volatile bool txReport = false;
volatile byte txPacket[DAVIS_PACKET_LEN];
volatile byte txChannel;
volatile uint32_t txInterval, txTarget;
volatile int16_t txPpm;

// clock calibration
int16_t calPpm = 0;        // this board's clock against the RFM69's crystal, 1/16 ppm
uint16_t calBursts = 0;    // calibrations applied
uint16_t calFails = 0;     // failed measurements
byte calPhase = 0;         // 0 idle, odd: long timeout next, even: short timeout next
byte calN;                 // measurements in the running burst
byte calBurstFails;        // failed measurements in the running burst
int16_t calVals[CAL_BURST];
uint32_t calLong;          // the long timeout of the running measurement, micros
uint32_t lastCal;          // end of the last burst, millis

// non-blocking LED blink, switched off from loop()
uint32_t ledOn, ledMs;

void blink(uint16_t ms) {
  if (LED < 0) return;
  digitalWrite(LED, HIGH);
  ledOn = millis();
  ledMs = ms;
}

void ledOff() {
  if (LED >= 0 && ledMs && millis() - ledOn >= ledMs) {
    digitalWrite(LED, LOW);
    ledMs = 0;
  }
}

void setup() {
  if (LED >= 0) pinMode(LED, OUTPUT);
  pinMode(POT_VCC, INPUT);        // hi-Z
  pinMode(POT_BLIND_VCC, INPUT);  // hi-Z
  pinMode(WIND_PIN, INPUT_PULLUP);
  Serial.begin(SERIAL_BAUD);
  delay(500);
  EEPROM.get(0, cfg);
  bool loaded = cfg.magic == CFG_MAGIC;
  if (!loaded) setDefaults();
  Serial.print(F("# "));
  Serial.print(NAME_VERSION);
  Serial.println(loaded ? F(" settings from EEPROM") : F(" default settings"));
#ifdef WIND_GATE_US
  Serial.print(F("# wind speed: rotations in a gate of "));
  Serial.print(WIND_GATE_US);
  Serial.println(F(" us"));
#endif
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), windInterrupt, CHANGE);
  startRadio();
  cmdShow();
}

void loop() {
  if (txReport) {
    printTPacket();
    blink(LED_TX_MS);
    windRead = false;
    // right after a transmission there is time for one timeout before the next one
    if (cfg.learn) calibrate();
  }
  if (!windRead && cfg.txId > 0 && radio.txTimeLeft() < WIND_LEAD_US) {
    readWind();
    windRead = true;
  }
  ledOff();
  processSerial();
}

void setDefaults() {
  cfg.magic = CFG_MAGIC;
  cfg.bandRaw = 1;    // EU
  cfg.txId = 3;
  cfg.learn = 1;      // works with a resonator and with a crystal
  cfg.ppm = 0;
  cfg.highPower = 1;  // RFM69HW/HCW
  cfg.frame = TX_FRAME_DAVIS;
}

byte band(byte raw) {
  return raw < FREQ_BAND_COUNT ? raw : FREQ_BAND_EU;  // the m codes are the Davis firmware's band numbers, as are the FREQ_BAND_* values
}

void startRadio() {
  radio.initialize(band(cfg.bandRaw));
  radio.stopReceiver();  // transmitter only, the radio sleeps between packets
  radio.setHighPower(cfg.highPower);
  radio.setTxFrame(cfg.frame);
  if (cfg.learn && calBursts == 0) {
    // a whole burst before the transmitter starts, so the first intervals are right already.
    // A band or ID change keeps the calibration: it doesn't depend on them
    calPhase = 0;
    do calibrate(); while (calPhase != 0 || (calBursts == 0 && calBurstFails < CAL_MAX_FAILS));
  }
  applyTiming();
  if (cfg.txId > 0) radio.enableTx(preparePacket, cfg.txId - 1);
}

void restartRadio() {
  radio.disableTx();
  startRadio();
}

void applyTiming() {
  int32_t ppm = (cfg.learn ? calPpm : 0) + (int32_t)cfg.ppm * TX_PPM_ONE;
  radio.setTxTiming(false, constrain(ppm, -TX_PPM_MAX, TX_PPM_MAX));
}

// ----- Clock calibration -----

// One step of a calibration burst: a long or a short timeout, whichever is next. A burst starts when
// CAL_PERIOD_S has passed since the last one, or right away while there is no calibration yet.
// This board's clock is fast by ppm if it counts more micros than the crystal's span; the transmit
// interval then needs that many more micros.
void calibrate() {
  if (calPhase == 0) {
    if (calBursts > 0 && millis() - lastCal < CAL_PERIOD_S * 1000UL) return;
    calPhase = 1;
    calN = calBurstFails = 0;
  }
  if (calPhase & 1) {
    calLong = radio.timeRxTimeout(CAL_BITRATE, CAL_N_LONG);
    radio.sleep();
    if (calLong == 0) {
      calFailed(0, 0);
      return;
    }
    calPhase++;
    return;
  }

  uint32_t ts = radio.timeRxTimeout(CAL_BITRATE, CAL_N_SHORT);
  radio.sleep();
  const int32_t nominal = (int32_t)(CAL_N_LONG - CAL_N_SHORT) * CAL_BITRATE / 2;
  const int32_t span = nominal + (int64_t)nominal * CAL_UNIT_PPM / 1000000;
  int32_t ppm = (ts == 0 || ts >= calLong) ? 0 : (int64_t)((int32_t)(calLong - ts) - span) * 16000000LL / span;
  if (ts == 0 || ts >= calLong || abs(ppm) > CAL_MAX_PPM * TX_PPM_ONE) {
    calFailed(calLong, ts);
    return;
  }
  calVals[calN++] = ppm;
  calPhase++;
  printK(calLong, ts);
  Serial.println(ppm / (float)TX_PPM_ONE, 1);
  if (calN >= CAL_BURST) calFinish();
}

// a failed measurement is repeated, until the burst has too many
void calFailed(uint32_t tl, uint32_t ts) {
  if (calFails < 0xffff) calFails++;
  calBurstFails++;
  calPhase = 1;
  printK(tl, ts);
  Serial.println(F("fail"));
  if (calBurstFails >= CAL_MAX_FAILS) calFinish();
}

// apply the median of the burst
void calFinish() {
  for (byte i = 1; i < calN; i++) {  // insertion sort
    int16_t v = calVals[i];
    byte j = i;
    for (; j > 0 && calVals[j - 1] > v; j--) calVals[j] = calVals[j - 1];
    calVals[j] = v;
  }
  bool ok = calN >= CAL_BURST / 2 + 1;
  if (ok) {
    calPpm = calN & 1 ? calVals[calN / 2] : (calVals[calN / 2 - 1] + calVals[calN / 2]) / 2;
    if (calBursts < 0xffff) calBursts++;
    applyTiming();
  }
  calPhase = 0;
  lastCal = millis();
  Serial.print(F("C "));
  if (ok) Serial.print(calPpm / (float)TX_PPM_ONE, 1); else Serial.print(F("fail"));
  Serial.print(' ');
  Serial.print(calN);
  Serial.print(' ');
  Serial.println(radio.txPpm() / (float)TX_PPM_ONE, 1);
}

void printK(uint32_t tl, uint32_t ts) {
  Serial.print(F("K "));
  Serial.print(tl);
  Serial.print(' ');
  Serial.print(ts);
  Serial.print(' ');
}

// ----- Wind -----

// The anemometer reed switch closes once per revolution. Both edges interrupt. The first edge after the switch
// has rested open for DEBOUNCE_US is a closure, whatever the level reads by now: a first contact can be over
// before the ISR reads the pin. Edges sooner after another (bounce on closing or opening) and the edge that ends
// a rest closed (opening) don't count. The level read after the last edge of a bounce burst is the settled one.
void windInterrupt() {
  uint32_t t = micros();
  if (lineOpen && t - lastEdge >= DEBOUNCE_US) rotation(t);
  lastEdge = t;
  lineOpen = digitalRead(WIND_PIN);
}

void rotation(uint32_t t) {
#ifdef WIND_GATE_US
  if (t - gateStart < WIND_GATE_US) gateCount++;
#endif
  if (pulseSeen && t - lastPulse <= WIND_CALM_US) {
    lastPeriod = t - lastPulse;
    windowCount++;
  } else {  // the first rotation after calm starts a new window
    windowStart = t;
    windowCount = 0;
    lastPeriod = 0;
    pulseSeen = true;
  }
  lastPulse = t;
}

// Wind speed over the rotations since the previous reading, and the vane direction
void readWind() {
#ifdef WIND_GATE_US
  uint32_t txLeft = radio.txTimeLeft();  // before noInterrupts(): it turns interrupts back on
#endif
  noInterrupts();
  uint32_t now = micros();
  bool seen = pulseSeen;
  uint32_t last = lastPulse, start = windowStart, period = lastPeriod;
  uint16_t count = windowCount;
  windowStart = last;  // the next window starts where this one ends, no rotation is lost
  windowCount = 0;
  if (now - lastEdge > 1000000UL) lastEdge = now - DEBOUNCE_US;  // at rest; micros wraps after 71 min
#ifdef WIND_GATE_US
  uint16_t gated = gateCount;
  gateCount = 0;
  // the next gate ends WIND_LEAD_US before the transmission after the coming one, where that reading is taken
  gateStart = now + txLeft + (40UL + cfg.txId) * 62500UL - WIND_LEAD_US - WIND_GATE_US;
  interrupts();
  uint32_t mph32 = (gated * (uint32_t)(WIND_RPS_MPH * 1000000UL) + WIND_GATE_US / 2) / WIND_GATE_US;
  windSpeed = mph32 > 255 ? 255 : mph32;
  windDir = readVane();
  updateGust();
  return;
#endif
  interrupts();

  uint32_t since = now - last;
  if (!seen || period == 0 || since > WIND_CALM_US) {
    period = 0;
  } else if (count > 0) {
    period = (last - start) / count;
  }
  if (period > 0 && since > period) period = since;  // slowing down: no pulse for longer than the last period
  uint16_t mph = period ? (uint16_t)(1000000.0 * WIND_RPS_MPH / period + 0.5) : 0;
  windSpeed = mph > 255 ? 255 : mph;
  windDir = readVane();
  updateGust();
}

// Track the highest speed of each gust period; before a gust packet, pick the highest of the last GUST_SLOTS
// periods (on a tie the newest; within a period the latest reading) and start a new period.
// seqIndex is the type of the coming packet: its ISR hasn't run yet.
void updateGust() {
  byte dir16 = ((windDir + 8) >> 4) & 0x0f;
  if (windSpeed >= gustMax[gustCur]) {
    gustMax[gustCur] = windSpeed;
    gustDir[gustCur] = dir16;
  }
  if (VP2_TX_SEQ[seqIndex] != VP2P_WINDGUST) return;
  byte best = gustCur, bestAge = 0;
  for (byte age = 1; age < GUST_SLOTS; age++) {
    byte s = (gustCur + GUST_SLOTS - age) % GUST_SLOTS;
    if (gustMax[s] > gustMax[best]) {
      best = s;
      bestAge = age;
    }
  }
  noInterrupts();
  gustSpeed = gustMax[best];
  gustInfo = (bestAge << 4) | gustDir[best];
  interrupts();
  gustCur = (gustCur + 1) % GUST_SLOTS;
  gustMax[gustCur] = gustDir[gustCur] = 0;
}

// Raw vane value: 1..255 along the potentiometer, 0 in its dead zone (north)
byte readVane() {
  int val, blindVal;

  pinMode(POT_BLIND_VCC, OUTPUT);
  digitalWrite(POT_BLIND_VCC, HIGH);
  delay(ANALOG_WAIT_MS);
  blindVal = analogRead(POT_PIN);
  pinMode(POT_VCC, OUTPUT);
  digitalWrite(POT_VCC, HIGH);
  pinMode(POT_BLIND_VCC, INPUT);
  delay(ANALOG_WAIT_MS);
  val = (analogReadAvg(POT_PIN, 4) + ANALOG_ROUNDING) >> 2;
  pinMode(POT_VCC, INPUT);
  if ((blindVal - val > GAP_DETECT_THRESH3) || (val < GAP_DETECT_THRESH1 && blindVal > GAP_DETECT_THRESH2)) {
    val = 0;
  }
  return val > 255 ? 255 : val;
}

int analogReadAvg(byte port, byte n) {
  long val = 0;
  for (byte i = 0; i < n; i++) val += analogRead(port);
  return val / n;
}

// ----- Transmission -----

// Called from the transmit ISR: fill the next packet of the sequence, CRC and transmitter ID are added by the library
void preparePacket(byte *packet) {
  byte type = VP2_TX_SEQ[seqIndex];
  if (++seqIndex >= sizeof(VP2_TX_SEQ)) seqIndex = 0;

  packet[0] = (type << 4) | radio.txId;
  packet[1] = windSpeed;
  packet[2] = windDir;
  for (byte i = 0; i < 3; i++) packet[3 + i] = VP2_NO_SENSOR[type][i];
  if (type == VP2P_WINDGUST) {
    packet[3] = gustSpeed;
    packet[4] = 0x01;
    packet[5] = gustInfo;
  }
  packet[6] = packet[7] = 0;
  packet[8] = packet[9] = 0xff;

  memcpy((void *)txPacket, packet, DAVIS_PACKET_LEN);
  txChannel = radio.txChannel;
  txInterval = radio.realTxDelay;
  txTarget = radio.txLastTarget;
  txPpm = radio.txPpm();
  txReport = true;
}

// T 3 80 05 2D FF C1 00 xx xx FF FF 2687000 2687016 -180.3 12 0 0.0
void printTPacket() {
  byte p[DAVIS_PACKET_LEN];
  noInterrupts();
  memcpy(p, (void *)txPacket, DAVIS_PACKET_LEN);
  byte ch = txChannel;
  uint32_t interval = txInterval, target = txTarget;
  int16_t ppm = txPpm;
  txReport = false;
  interrupts();
  uint16_t crc = radio.crc16_ccitt(p, 6);
  p[6] = crc >> 8;
  p[7] = crc & 0xff;

  Serial.print(F("T "));
  Serial.print(ch);
  Serial.print(' ');
  printHex(p, DAVIS_PACKET_LEN);
  Serial.print(' ');
  Serial.print(interval);
  Serial.print(' ');
  Serial.print(target);
  Serial.print(' ');
  Serial.print(ppm / (float)TX_PPM_ONE, 1);
  Serial.print(' ');
  Serial.print(calBursts);
  Serial.print(' ');
  Serial.print(calFails);
  Serial.println(F(" 0.0"));
}

void printHex(volatile byte *packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
    Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print(' ');
  }
}

void printStatus(bool st, const __FlashStringHelper *info = NULL) {
  Serial.print(st ? F("# OK") : F("# ERR"));
  if (info != NULL) {
    Serial.print(' ');
    Serial.print(info);
  }
  Serial.println();
}

// ----- Commands -----

void processSerial() {
  static byte i;
  static char s[SERBUF_LEN];

  if (!Serial.available()) return;
  int c = Serial.read();

  if (c == '\r' || c == '\n') {
    if (i == 0) return;  // empty line, or the second char of CR LF
    s[i] = 0;
    i = 0;
    if (localEcho) Serial.println();
    command(s);
  } else {
    if (localEcho) Serial.print((char)c);
    if (i < SERBUF_LEN - 1) {
      s[i++] = c;
    } else {
      i = 0;
      Serial.println();
      printStatus(S_ERR, F("bad command"));
    }
  }
}

void command(char *s) {
  char *a = s + 1;
  bool num = isdigit(a[0]) || (a[0] == '-' && isdigit(a[1]));
  long v = atol(a);

  switch (s[0]) {
    case 'm':
      if (!num || v < 0 || v >= FREQ_BAND_COUNT) break;
      cfg.bandRaw = v;
      restartRadio();
      cmdShow();
      return;

    case 'i':
      if (!num || v < 0 || v > 8) break;
      cfg.txId = v;
      restartRadio();
      cmdShow();
      return;

    case 'c':
      if (!num || v < 0 || v > 1) break;
      cfg.learn = v;
      restartRadio();
      cmdShow();
      return;

    case 'p':
      if (!num || v < -TX_PPM_MAX / TX_PPM_ONE || v > TX_PPM_MAX / TX_PPM_ONE) break;
      cfg.ppm = v;
      applyTiming();
      cmdShow();
      return;

    case 'h':
      if (!num || v < 0 || v > 1) break;
      cfg.highPower = v;
      restartRadio();
      cmdShow();
      return;

    case 'g':
      if (!num || v < 0 || v > 1) break;
      cfg.frame = v;
      radio.setTxFrame(v);
      cmdShow();
      return;

    case 'e':
      EEPROM.put(0, cfg);
      printStatus(S_OK, F("saved"));
      return;

    case 'd':
      setDefaults();
      restartRadio();
      cmdShow();
      return;

    case 'r':
      printStatus(S_OK, F("resetting"));
      Serial.flush();
      radio.disableTx();
      radio.sleep();
      wdt_enable(WDTO_15MS);
      for (;;) {}

    case '?':
      cmdShow();
      return;
  }
  printStatus(S_ERR, F("bad command"));
}

// # OK m1 i3 c1 p0 h1 g1
void cmdShow() {
  Serial.print(F("# OK m"));
  Serial.print(cfg.bandRaw);
  Serial.print(F(" i"));
  Serial.print(cfg.txId);
  Serial.print(F(" c"));
  Serial.print(cfg.learn);
  Serial.print(F(" p"));
  Serial.print(cfg.ppm);
  Serial.print(F(" h"));
  Serial.print(cfg.highPower);
  Serial.print(F(" g"));
  Serial.print(cfg.frame);
  Serial.println();
}

// clear the watchdog after the reset it caused, or it keeps resetting
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

void wdt_init(void) {
  MCUSR = 0;
  wdt_disable();
  return;
}
