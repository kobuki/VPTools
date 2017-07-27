#include <Arduino.h>
#include <DavisRFM69.h>

#define TX_ID 2 // 0..7, Davis transmitter ID, set to a different value than all other transmitters
                // IMPORTANT: set it ONE LESS than you'd set it on the ISS via the DIP switch; 1 here is 2 on the ISS/Davis console

// Transmitter timing constants (for 2 of my own Davis transmitters),
// can be used to adapt to local XCO/resonator frequency errors
// iss: 0.998787 -> setTimeBase(998787)
// atk: 0.997647 -> setTimeBase(997647)

// Observed sequence of transmitted VP2 ISS value types.
// The upper nibble is important, the lower nibble is the transmitter ID + battery flag.
// Type values for a standard VP2 ISS:
//   0x80  0xe0  0x40  0xa0  0x60  0x50  0x90
//   temp  rain  uv    rh    sol   rsecs gust
// Wind speed and direction is transmitted in every packet at byte offsets 1 and 2.
static const byte txseq_vp2[20] =
{
  0x80, 0xe0, 0x50, 0x40,
  0x80, 0xe0, 0x50, 0x90,
  0x80, 0xe0, 0x50, 0xa0,
  0x80, 0xe0, 0x50, 0xa0,
  0x80, 0xe0, 0x50, 0x60
};

DavisRFM69 radio;
byte seqIndex; // current packet type index in txseq_vp2
byte vaneAngleRaw = 0, windSpeed = 10;
byte txChannel = 0;
bool txDataAvailable = false;
volatile static byte txPacket[DAVIS_PACKET_LEN];

// id, type, active
Station stations[2] = {
  { 0, STYPE_ISS,         true },
  { 1, STYPE_WLESS_ANEMO, true },
};

void setup() {
  delay(1000);
  Serial.begin(115200);
  radio.setStations(stations, 2);
  // radio.setTimeBase(1000000);
  radio.initialize(FREQ_BAND_EU);
  radio.enableTx(preparePacket, TX_ID);
  seqIndex = 0;
}

void loop() {

    if (txDataAvailable) {
      txDataAvailable = false;
  
      uint16_t crc = radio.crc16_ccitt((volatile byte *)txPacket, 6);
      txPacket[6] = crc >> 8;
      txPacket[7] = crc & 0xff;
  
      byte a = pgm_read_byte(&bandTab[radio.band][txChannel][0]);
      byte b = pgm_read_byte(&bandTab[radio.band][txChannel][1]);
      byte c = pgm_read_byte(&bandTab[radio.band][txChannel][2]);
      double freq = (((uint32_t)a<<16) | ((uint32_t)b<<8) | (uint32_t)c) * RF69_FSTEP / 1000000.0;
    
      print_value("T angle", vaneAngleRaw);
      print_value("speed", windSpeed);
      Serial.print("raw:");
      printHex(txPacket, 10);
      print_value(" txdelay", (int32_t)radio.realTxDelay);
      print_value("channel", txChannel);
      Serial.print("freq:");
      Serial.print(freq, 3);
      print_value(" freeram", getFreeRam());
      Serial.println();

      vaneAngleRaw = random(45, 75);
      windSpeed = random(10, 20);
    }

    if (radio.fifo.hasElements()) {
      RadioData* rd = radio.fifo.dequeue();
      byte* p = rd->packet;

      Serial.print("R raw:");
      printHex(p, 10);
      print_value(" station", p[0] & 0x7);
      Serial.print("packets:");
      Serial.print(radio.packets);
      Serial.print('/');
      Serial.print(radio.lostPackets);
      Serial.print('/');
      Serial.print((float)(radio.packets * 100.0 / (radio.packets + radio.lostPackets)));
      print_value(" channel", rd->channel);
      print_value("rssi", -rd->rssi);
      print_value("batt", (char*)(p[0] & 0x8 ? "err" : "ok"));
      print_value("fei", round(rd->fei * RF69_FSTEP / 1000));
      print_value("delta", rd->delta);
      Serial.println();
    }
}

// fill transmitted packet payload here, use proper type sequence; channel hopping is automatic
// do NOT prepare values here, use values prepared in loop(), etc.
// called from an ISR so must be as fast as possible
void preparePacket(byte* packet) {
  packet[0] = txseq_vp2[seqIndex] | TX_ID;
  if (++seqIndex >= sizeof(txseq_vp2)) seqIndex = 0;
  // dummy data
  packet[1] = windSpeed;
  packet[2] = vaneAngleRaw;
  packet[3] = packet[4] = packet[5] = 0xf0;
  packet[8] = packet[9] = 0xff;
  txChannel = radio.txChannel;
  memcpy((void*)txPacket, packet, 10);
  txDataAvailable = true;
}

void printHex(volatile byte* packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
    Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print('-');
  }
}

int16_t getFreeRam() {
  extern int __heap_start, *__brkval;
  int16_t v;
  return (int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval);
}

void print_value(char* vname, char* value) {
  Serial.print(vname); Serial.print(':'); Serial.print(value); Serial.print(' ');
}

void print_value(char* vname, int value) {
  Serial.print(vname); Serial.print(':'); Serial.print(value); Serial.print(' ');
}

void print_value(char* vname, float value) {
  Serial.print(vname); Serial.print(':'); Serial.print(value, 1); Serial.print(' ');
}

void print_value(char* vname, long value) {
  Serial.print(vname); Serial.print(':'); Serial.print(value); Serial.print(' ');
}

void print_value(char* vname, uint32_t value) {
  Serial.print(vname); Serial.print(':'); Serial.print(value); Serial.print(' ');
}

