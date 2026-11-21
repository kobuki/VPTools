// RXTX: receive several Davis transmitters and retransmit their data mixed into a single ISS packet stream.
//
// A Vantage console or Envoy takes each sensor from a fixed transmitter type, so data from different
// transmitters (eg. wind from an anemometer transmitter, everything else from an ISS) can't always be combined
// there. This sketch picks every packet type from a configurable source station and sends the result as one
// VP2 ISS. A type whose source hasn't been heard for a while is sent with Davis' "no sensor" values.
//
// Timing and frequency follow the received stations, so no per-board calibration is needed:
// - the transmit interval gets the same error that this board's clock shows on the received stations' intervals,
// - the transmit frequency gets the software AFC's learned offset of the received stations.
//
// Output: I lines as WxReceiver's (received packets), T lines for transmitted packets:
//   T <channel> <10 bytes> <interval> <target> <timing ppm> <samples> <rejects> <tx freq corr kHz>
// where interval is the transmit interval that just ended and target the one it was scheduled for (micros).
//
// Commands (end with CR or LF; settings take effect at once, e saves them to EEPROM):
//   m<0|1|2|5>  band: US, EU, AU, NZ
//   t<mask>     stations to receive: bit 0 = ID 1 ... bit 7 = ID 8
//   i<0..8>     transmitter ID to send as, 0 = transmitter off (receive only)
//   x<raw>      RSSI threshold, raw (190 = -95 dBm)
//   a<n|h>      AFC: 0 off, 1..50 software AFC limited to +/- n kHz, h RFM69 HW AFC
//   c<0|1>      transmit timing: 0 fixed at p, 1 learned from the received stations, starting at p
//   p<ppm>      transmit timing correction (ppm, integer)
//   f<0|1>      transmit frequency correction from the software AFC
//   w<0..8>     wind source station ID (0 = no wind sensor)
//   s<T><0..8>  source station ID for packet type T (hex nibble of byte 0, eg. s92 = gust from ID 2)
//   n<s>        seconds without data from a source before its "no sensor" values are sent
//   h<0|1>      RFM69 module: 0 W/CW (PA0, +13 dBm), 1 HW/HCW (PA_BOOST, +20 dBm); a mismatch transmits only leakage
//   e           save settings to EEPROM
//   d           load the defaults (not saved)
//   r           reset
//   ?           show settings

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#include "DavisRFM69.h"
#include "PacketFifo.h"

#define NAME_VERSION F("RXTX v2026112101")
#define SERIAL_BAUD 115200
#define SERBUF_LEN 8
#define CFG_MAGIC 0x5255  // change when Config changes

#define S_OK true
#define S_ERR false

struct Config {
  uint16_t magic;
  byte bandRaw;       // m
  byte stations;      // t
  byte txId;          // i, 1..8, 0 = off
  byte rssi;          // x
  byte afc;           // a
  byte learn;         // c
  int16_t ppm;        // p
  byte txAfc;         // f
  byte windSrc;       // w, 0..8
  byte typeSrc[16];   // s, 0..8 per packet type
  byte timeout;       // n
  byte highPower;     // h
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

// "No sensor" values of bytes 3..5 for each packet type; wind (bytes 1, 2) is 0 0
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

// latest data from the sources, written in loop(), read in the transmit ISR
struct Slot {
  byte v[3];
  bool batt;
  bool valid;
  uint32_t ms;
};

Slot slots[16];  // bytes 3..5 per packet type
Slot wind;       // bytes 1, 2

DavisRFM69 radio;
Station stations[8];
byte numStations;
bool localEcho = false;

byte seqIndex = 0;

// transmit report, filled in the transmit ISR
volatile bool txReport = false;
volatile byte txPacket[DAVIS_PACKET_LEN];
volatile byte txChannel;
volatile uint32_t txInterval, txTarget;
volatile int16_t txPpm;
volatile uint16_t txSamples, txRejects;
volatile int16_t txAfcCorr;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  EEPROM.get(0, cfg);
  bool loaded = cfg.magic == CFG_MAGIC;
  if (!loaded) setDefaults();
  Serial.print(F("# "));
  Serial.print(NAME_VERSION);
  Serial.println(loaded ? F(" settings from EEPROM") : F(" default settings"));
  radio.setTxTiming(cfg.learn, cfg.ppm * TX_PPM_ONE);
  startRadio();
  cmdShow();
}

void loop() {
  if (radio.fifo.hasElements()) {
    RadioData *rd = radio.fifo.dequeue();
    store(rd->packet);
    printIPacket(rd);
  }
  if (txReport) printTPacket();
  processSerial();
}

void setDefaults() {
  cfg.magic = CFG_MAGIC;
  cfg.bandRaw = 1;    // EU
  cfg.stations = 3;   // IDs 1 and 2
  cfg.txId = 3;
  cfg.rssi = 190;
  cfg.afc = 12;
  cfg.learn = 1;
  cfg.ppm = 0;
  cfg.txAfc = 1;
  cfg.windSrc = 2;
  for (byte i = 0; i < 16; i++) cfg.typeSrc[i] = 1;
  cfg.typeSrc[0x9] = 2;  // gust goes with the wind
  cfg.typeSrc[0x4] = 2;  // UV and solar are on the anemometer transmitter's SIM board here
  cfg.typeSrc[0x6] = 2;
  cfg.timeout = 180;     // the rarest types come every 20 packets, ~52 s
  cfg.highPower = 1;     // RFM69HW/HCW
}

byte band(byte raw) {
  switch (raw) {
    case 0: return FREQ_BAND_US;
    case 2: return FREQ_BAND_AU;
    case 5: return FREQ_BAND_NZ;
    default: return FREQ_BAND_EU;
  }
}

void startRadio() {
  numStations = 0;
  for (byte i = 0; i < 8; i++) {
    if (cfg.stations & (1 << i)) {
      stations[numStations].id = i;
      stations[numStations].type = STYPE_ISS;
      stations[numStations].active = true;
      numStations++;
    }
  }
  radio.initialize(band(cfg.bandRaw));
  radio.setHighPower(cfg.highPower);
  radio.setStations(stations, numStations);
  radio.setRssiThresholdRaw(cfg.rssi);
  radio.setBandwidth(RF69_DAVIS_BW_WIDE);
  radio.setAfc(cfg.afc);
  radio.setTxAfc(cfg.txAfc);
  if (cfg.txId > 0) radio.enableTx(preparePacket, cfg.txId - 1);
}

void restartRadio() {
  radio.disableTx();
  radio.stopReceiver();
  radio.fifo.flush();
  startRadio();
}

// ----- Mixing -----

// keep the parts of a received packet that are configured to come from its station
void store(byte *p) {
  byte id = (p[0] & 7) + 1;
  byte type = p[0] >> 4;
  bool batt = p[0] & 8;
  uint32_t now = millis();
  noInterrupts();
  if (cfg.typeSrc[type] == id) {
    for (byte i = 0; i < 3; i++) slots[type].v[i] = p[3 + i];
    slots[type].batt = batt;
    slots[type].valid = true;
    slots[type].ms = now;
  }
  if (cfg.windSrc == id) {
    wind.v[0] = p[1];
    wind.v[1] = p[2];
    wind.batt = batt;
    wind.valid = true;
    wind.ms = now;
  }
  interrupts();
}

bool fresh(Slot *s, uint32_t now) {
  return s->valid && now - s->ms < cfg.timeout * 1000UL;
}

// Called from the transmit ISR: fill the next packet of the sequence, CRC and transmitter ID are added by the library
void preparePacket(byte *packet) {
  uint32_t now = millis();
  byte type = txseq_vp2[seqIndex];
  if (++seqIndex >= sizeof(txseq_vp2)) seqIndex = 0;

  bool batt = false;
  if (fresh(&wind, now)) {
    packet[1] = wind.v[0];
    packet[2] = wind.v[1];
    batt |= wind.batt;
  } else {
    packet[1] = packet[2] = 0;
  }
  Slot *s = &slots[type];
  for (byte i = 0; i < 3; i++) packet[3 + i] = fresh(s, now) ? s->v[i] : noSensor[type][i];
  if (fresh(s, now)) batt |= s->batt;
  packet[0] = (type << 4) | (batt ? 8 : 0) | radio.txId;
  packet[6] = packet[7] = 0;
  packet[8] = packet[9] = 0xff;

  memcpy((void *)txPacket, packet, DAVIS_PACKET_LEN);
  txChannel = radio.txChannel;
  txInterval = radio.realTxDelay;
  txTarget = radio.txLastTarget;
  txPpm = radio.txPpm();
  txSamples = radio.txPpmSamples;
  txRejects = radio.txPpmRejects;
  txAfcCorr = radio.txAfcApplied;
  txReport = true;
}

// ----- Output -----

// I 104 61 00 00 FF C3 00 96 DC  -64 2624972 2
void printIPacket(RadioData *rd) {
  Serial.print(F("I 1"));
  if (rd->channel < 10) Serial.print('0');
  Serial.print(rd->channel);
  Serial.print(' ');
  printHex(rd->packet, 8);
  Serial.print(F("  "));
  Serial.print(-rd->rssi);
  Serial.print(' ');
  Serial.print(rd->delta);
  Serial.print(' ');
  Serial.println(round(rd->fei * RF69_FSTEP / 1000));
}

// T 3 80 05 2D FF C1 00 xx xx FF FF 2687000 2687016 -180.3 123 2 -8.5
void printTPacket() {
  byte p[DAVIS_PACKET_LEN];
  noInterrupts();
  memcpy(p, (void *)txPacket, DAVIS_PACKET_LEN);
  byte ch = txChannel;
  uint32_t interval = txInterval, target = txTarget;
  int16_t ppm = txPpm, afc = txAfcCorr;
  uint16_t samples = txSamples, rejects = txRejects;
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
  Serial.print(samples);
  Serial.print(' ');
  Serial.print(rejects);
  Serial.print(' ');
  Serial.println(afc * RF69_FSTEP / 1000, 1);
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

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  c = toupper(c);
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

void command(char *s) {
  char *a = s + 1;
  bool num = isdigit(a[0]) || (a[0] == '-' && isdigit(a[1]));
  long v = atol(a);

  switch (s[0]) {
    case 'm':
      if (!num || !(v == 0 || v == 1 || v == 2 || v == 5)) break;
      cfg.bandRaw = v;
      restartRadio();
      cmdShow();
      return;

    case 't':
      if (!num || v < 1 || v > 255) break;
      cfg.stations = v;
      restartRadio();
      cmdShow();
      return;

    case 'i':
      if (!num || v < 0 || v > 8) break;
      cfg.txId = v;
      restartRadio();
      cmdShow();
      return;

    case 'x':
      if (!num || v < 1 || v > 255) break;
      cfg.rssi = v;
      radio.setRssiThresholdRaw(v);
      cmdShow();
      return;

    case 'a':
      if (a[0] == 'h') v = RF69_AFC_HW;
      else if (!num || v < 0 || v > RF69_AFC_MAX) break;
      cfg.afc = v;
      radio.setAfc(v);
      cmdShow();
      return;

    case 'c':
      if (!num || v < 0 || v > 1) break;
      cfg.learn = v;
      radio.setTxTiming(cfg.learn, cfg.ppm * TX_PPM_ONE);
      cmdShow();
      return;

    case 'p':
      if (!num || v < -TX_PPM_MAX / TX_PPM_ONE || v > TX_PPM_MAX / TX_PPM_ONE) break;
      cfg.ppm = v;
      radio.setTxTiming(cfg.learn, cfg.ppm * TX_PPM_ONE);
      cmdShow();
      return;

    case 'f':
      if (!num || v < 0 || v > 1) break;
      cfg.txAfc = v;
      radio.setTxAfc(v);
      cmdShow();
      return;

    case 'w':
      if (!num || v < 0 || v > 8) break;
      noInterrupts();
      cfg.windSrc = v;
      wind.valid = false;
      interrupts();
      cmdShow();
      return;

    case 's': {
      int type = hexNibble(a[0]);
      if (type < 0 || !isdigit(a[1]) || a[2] != 0 || a[1] > '8') break;
      noInterrupts();
      cfg.typeSrc[type] = a[1] - '0';
      slots[type].valid = false;
      interrupts();
      cmdShow();
      return;
    }

    case 'n':
      if (!num || v < 1 || v > 255) break;
      cfg.timeout = v;
      cmdShow();
      return;

    case 'h':
      if (!num || v < 0 || v > 1) break;
      cfg.highPower = v;
      restartRadio();
      cmdShow();
      return;

    case 'e':
      EEPROM.put(0, cfg);
      printStatus(S_OK, F("saved"));
      return;

    case 'd':
      setDefaults();
      radio.setTxTiming(cfg.learn, cfg.ppm * TX_PPM_ONE);
      restartRadio();
      cmdShow();
      return;

    case 'r':
      printStatus(S_OK, F("resetting"));
      Serial.flush();
      radio.disableTx();
      radio.stopReceiver();
      wdt_enable(WDTO_15MS);
      for (;;) {}

    case '?':
      cmdShow();
      return;
  }
  printStatus(S_ERR, F("bad command"));
}

// # OK m1 t3 i3 x190 a12 c1 p0 f1 w2 n180 h1 s 4:2 5:1 6:2 8:1 9:2 A:1 E:1
void cmdShow() {
  Serial.print(F("# OK m"));
  Serial.print(cfg.bandRaw);
  Serial.print(F(" t"));
  Serial.print(cfg.stations);
  Serial.print(F(" i"));
  Serial.print(cfg.txId);
  Serial.print(F(" x"));
  Serial.print(cfg.rssi);
  Serial.print(F(" a"));
  if (cfg.afc == RF69_AFC_HW) Serial.print('h'); else Serial.print(cfg.afc);
  Serial.print(F(" c"));
  Serial.print(cfg.learn);
  Serial.print(F(" p"));
  Serial.print(cfg.ppm);
  Serial.print(F(" f"));
  Serial.print(cfg.txAfc);
  Serial.print(F(" w"));
  Serial.print(cfg.windSrc);
  Serial.print(F(" n"));
  Serial.print(cfg.timeout);
  Serial.print(F(" h"));
  Serial.print(cfg.highPower);
  Serial.print(F(" s"));
  // the packet types the VP2 sequence sends
  static const byte types[] = { 0x4, 0x5, 0x6, 0x8, 0x9, 0xa, 0xe };
  for (byte i = 0; i < sizeof(types); i++) {
    Serial.print(' ');
    Serial.print(types[i], HEX);
    Serial.print(':');
    Serial.print(cfg.typeSrc[types[i]]);
  }
  Serial.println();
}

// clear the watchdog after the reset it caused, or it keeps resetting
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

void wdt_init(void) {
  MCUSR = 0;
  wdt_disable();
  return;
}
