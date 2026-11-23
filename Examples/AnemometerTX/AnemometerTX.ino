// AnemometerTX: read a Davis anemometer and transmit it as a Davis transmitter (the ISS SIM board).
//
// The packets follow the VP2 ISS type sequence and timing. Wind speed and direction go in every packet,
// the other packet types carry Davis' "no sensor" values. The radio sleeps between transmissions.
//
// Transmit timing: a Davis receiver expects the interval (40 + ID) / 16 s. A board with a quartz crystal can run
// on its own clock (c0, with a fixed correction p). A Moteino's ceramic resonator is too far off and drifts
// with temperature, so with c1 the board measures its clock against the RFM69's crystal every CAL_PERIOD_S,
// using the RFM69's RX timeout (see DavisRFM69::timeRxTimeout()), and corrects the interval with that.
//
// Hardware (Moteino or compatible ATmega328P with an RFM69):
//   anemometer reed switch between WIND_PIN (INT1) and GND
//   wind vane potentiometer: one end on POT_VCC, wiper on POT_PIN, other end on GND;
//   POT_BLIND_VCC through an extra resistor to the POT_VCC end, for detecting the dead zone
//
// Output: T lines for transmitted packets, as RXTX's:
//   T <channel> <10 bytes> <interval> <target> <timing ppm> <calibrations> <rejects> 0.0
// where interval is the transmit interval that just ended and target the one it was scheduled for (micros),
// and K lines for clock calibrations (c1):
//   K <long us> <short us> <measured ppm> <timing ppm> <ok|reject|fail>
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

#define NAME_VERSION F("AnemometerTX v2026112301")
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

#define DEBOUNCE_US 5000UL        // reed switch bounce
#define WIND_CALM_US 2250000UL    // no rotation for this long is calm (below 1 mph)
#define WIND_RPS_MPH 2.25         // 1 revolution per second is 2.25 mph (Davis)
#define WIND_LEAD_US 100000UL     // read the sensors this long before a transmission

// ----- Clock calibration (c1) -----

#define CAL_PERIOD_S 60       // seconds between calibrations
#define CAL_BITRATE 9800      // RFM69 bitrate register: bit time 306.25 us, timeout unit 4.9 ms
#define CAL_N_LONG 255        // 1.2495 s
#define CAL_N_SHORT 63        // 0.3087 s; the difference, 0.9408 s, is the measured span
#define CAL_GATE (200L * TX_PPM_ONE)  // once settled, measurements further than this from the estimate are rejected
#define CAL_REJECT_RUN 3      // this many rejects in a row mean the estimate itself is wrong: start over
#define CAL_SLOW 4            // after the first measurement each one moves the estimate 1/CAL_SLOW of the way

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

// VP2 ISS packet type sequence (upper nibble of byte 0).
// Wind speed and direction are in every packet at byte offsets 1 and 2.
static const byte txseq_vp2[20] =
{
  0x8, 0xe, 0x5, 0x4,
  0x8, 0xe, 0x5, 0x9,
  0x8, 0xe, 0x5, 0xa,
  0x8, 0xe, 0x5, 0xa,
  0x8, 0xe, 0x5, 0x6
};

// "No sensor" values of bytes 3..5 for each packet type
static const byte noSensor[16][3] =
{
  { 0x00, 0x00, 0x00 },  // 0
  { 0x00, 0x00, 0x00 },  // 1
  { 0x00, 0x00, 0x00 },  // 2
  { 0x00, 0x00, 0x00 },  // 3
  { 0xff, 0xc5, 0x00 },  // 4 UV
  { 0xff, 0x71, 0x00 },  // 5 rain rate
  { 0xff, 0xc5, 0x00 },  // 6 solar
  { 0x00, 0x00, 0x00 },  // 7
  { 0xff, 0xc1, 0x00 },  // 8 temperature
  { 0x00, 0x05, 0x00 },  // 9 gust
  { 0x00, 0x01, 0x00 },  // A humidity
  { 0x00, 0x00, 0x00 },  // B
  { 0x00, 0x05, 0x00 },  // C unknown
  { 0x00, 0x00, 0x00 },  // D
  { 0x80, 0x01, 0x00 },  // E rain
  { 0x00, 0x00, 0x00 },  // F
};

DavisRFM69 radio;
bool localEcho = false;

// anemometer pulses, written in the wind ISR
volatile uint32_t lastPulse;      // time of the last debounced pulse, micros
volatile uint32_t lastEdge;       // time of the last edge, bounces included
volatile uint32_t lastPeriod;     // the last rotation period, 0 if unknown
volatile uint32_t windowStart;    // the pulse the averaging window starts at
volatile uint16_t windowCount;    // rotations since windowStart
volatile bool pulseSeen = false;

// wind data for the next packet, written in loop(), read in the transmit ISR
volatile byte windSpeed = 0;      // mph
volatile byte windDir = 0;        // raw vane value, 0 in the dead zone
bool windRead = false;            // read for the coming transmission

byte seqIndex = 0;

// transmit report, filled in the transmit ISR
volatile bool txReport = false;
volatile byte txPacket[DAVIS_PACKET_LEN];
volatile byte txChannel;
volatile uint32_t txInterval, txTarget;
volatile int32_t txPpm;

// clock calibration
int32_t calPpm;            // estimate of this board's clock against the RFM69's, 1/16 ppm
uint16_t calSamples = 0;
uint16_t calRejects = 0;
byte calRejectRun = 0;     // rejects in a row; after CAL_REJECT_RUN the estimate starts over
uint32_t lastCal;          // millis

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
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), windInterrupt, FALLING);
  startRadio();
  cmdShow();
}

void loop() {
  if (txReport) {
    printTPacket();
    blink(LED_TX_MS);
    windRead = false;
    // right after a transmission there is time for a calibration before the next one
    if (cfg.learn && millis() - lastCal >= CAL_PERIOD_S * 1000UL) calibrate();
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
  if (cfg.learn) {
    calSamples = calRejects = calRejectRun = 0;
    calibrate();  // before the transmitter starts, so the first intervals are right already
  }
  applyTiming();
  if (cfg.txId > 0) radio.enableTx(preparePacket, cfg.txId - 1);
}

void restartRadio() {
  radio.disableTx();
  startRadio();
}

void applyTiming() {
  radio.setTxTiming(false, (cfg.learn ? calPpm : 0) + (int32_t)cfg.ppm * TX_PPM_ONE);
}

// ----- Clock calibration -----

// Two RX timeouts of different length: the difference cancels the RFM69's start-up delay.
// This board's clock is fast by ppm if it counts more micros than the crystal's span; the transmit
// interval then needs that many more micros.
void calibrate() {
  lastCal = millis();
  uint32_t tl = radio.timeRxTimeout(CAL_BITRATE, CAL_N_LONG);
  uint32_t ts = tl ? radio.timeRxTimeout(CAL_BITRATE, CAL_N_SHORT) : 0;
  radio.sleep();
  int32_t ppm = 0;
  const __FlashStringHelper *result;
  if (tl == 0 || ts == 0 || ts >= tl) {
    result = F("fail");
  } else {
    int32_t span = (int32_t)(CAL_N_LONG - CAL_N_SHORT) * CAL_BITRATE / 2;
    ppm = (int64_t)((int32_t)(tl - ts) - span) * 16000000LL / span;
    if (calSamples > 0 && abs(ppm - calPpm) > CAL_GATE && ++calRejectRun < CAL_REJECT_RUN) {
      if (calRejects < 0xffff) calRejects++;
      result = F("reject");
    } else {
      if (calRejectRun >= CAL_REJECT_RUN) calSamples = 0;
      calRejectRun = 0;
      calPpm = calSamples == 0 ? ppm : calPpm + (ppm - calPpm) / CAL_SLOW;
      if (calSamples < 0xffff) calSamples++;
      applyTiming();
      result = F("ok");
    }
  }
  Serial.print(F("K "));
  Serial.print(tl);
  Serial.print(' ');
  Serial.print(ts);
  Serial.print(' ');
  Serial.print(ppm / (float)TX_PPM_ONE, 1);
  Serial.print(' ');
  Serial.print(calPpm / (float)TX_PPM_ONE, 1);
  Serial.print(' ');
  Serial.println(result);
}

// ----- Wind -----

// The anemometer reed switch closes once per revolution.
void windInterrupt() {
  uint32_t t = micros();
  if (t - lastEdge > DEBOUNCE_US) {
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
  lastEdge = t;
}

// Wind speed over the rotations since the previous reading, and the vane direction
void readWind() {
  noInterrupts();
  uint32_t now = micros();
  bool seen = pulseSeen;
  uint32_t last = lastPulse, start = windowStart, period = lastPeriod;
  uint16_t count = windowCount;
  windowStart = last;  // the next window starts where this one ends, no rotation is lost
  windowCount = 0;
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
  byte type = txseq_vp2[seqIndex];
  if (++seqIndex >= sizeof(txseq_vp2)) seqIndex = 0;

  packet[0] = (type << 4) | radio.txId;
  packet[1] = windSpeed;
  packet[2] = windDir;
  for (byte i = 0; i < 3; i++) packet[3 + i] = noSensor[type][i];
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
  int32_t ppm = txPpm;
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
  Serial.print(calSamples);
  Serial.print(' ');
  Serial.print(calRejects);
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
