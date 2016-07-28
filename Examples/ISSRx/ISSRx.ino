#include <Arduino.h>

#include <SPI.h>
#include <EEPROM.h>

#include <Wire.h>

#include "DavisRFM69.h"
#include "PacketFifo.h"

#define LED 9 // Moteinos have LEDs on D9
#define SERIAL_BAUD 115200

DavisRFM69 radio;

// id, type, active
Station stations[3] = {
  { 0, STYPE_ISS,         true },
  { 1, STYPE_WLESS_ANEMO, true },
  { 2, STYPE_TEMP_ONLY,   true },
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.setStations(stations, 3);
  radio.initialize(FREQ_BAND_EU);
  radio.setBandwidth(RF69_DAVIS_BW_NARROW);
}

void loop() {
  if (radio.fifo.hasElements()) {
    decode_packet(radio.fifo.dequeue());
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

void print_value(char* vname, char* value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(' ');
}

void print_value(char* vname, int value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(' ');
}

void print_value(char* vname, float value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value, 1); Serial.print(' ');
}

void print_value(char* vname, long value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(' ');
}

void print_value(char* vname, uint32_t value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(' ');
}

void decode_packet(RadioData* rd) {
  int val;
  byte* packet = rd->packet;

  Serial.print(F("raw:"));
  printHex(packet, 10);
  print_value(" station", packet[0] & 0x7);

  Serial.print(F("packets:"));
  Serial.print(radio.packets);
  Serial.print('/');
  Serial.print(radio.lostPackets);
  Serial.print('/');
  Serial.print((float)(radio.packets * 100.0 / (radio.packets + radio.lostPackets)));
  Serial.print(' ');

  print_value("channel", rd->channel);
  print_value("rssi", -rd->rssi);

  print_value("batt", (char*)(packet[0] & 0x8 ? "err" : "ok"));

  // All packet payload values are printed unconditionally, either properly
  // calculated or flagged with a special "missing sensor" value, mostly -1.
  // It's the CPE's responsibility to interpret our output accordingly.

  byte id = radio.DATA[0] & 7;
  int stIx = radio.findStation(id);

  // wind data is present in every packet, windd == 0 (packet[2] == 0) means there's no anemometer
  if (packet[2] != 0) {
    if (stations[stIx].type == STYPE_VUE) {
      val = (packet[2] << 1) | (packet[4] & 2) >> 1;
      val = round(val * 360 / 512);
    } else {
      val = 9 + round((packet[2] - 1) * 342.0 / 255.0);
    }
  } else {
    val = 0;
  }
  print_value("windv", packet[1]);
  print_value("winddraw", packet[2]);
  print_value("windd", val);

  switch (packet[0] >> 4) {

    case VP2P_UV:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3ff) {
        print_value("uv", (float)(val / 50.0));
      } else {
        print_value("uv", -1);
      }
      break;

    case VP2P_SOLAR:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3fe) {
        print_value("solar", (float)(val * 1.757936));
      } else {
        print_value("solar", -1);
      }
      break;

    case VP2P_RAIN:
      if (packet[3] == 0x80) {
        print_value("rain", -1);
      } else {
        print_value("rain", packet[3]);
      }
      break;

    case VP2P_RAINSECS:
      // light rain:  byte4[5:4] as value[9:8] and byte3[7:0] as value[7:0] - 10 bits total
      // strong rain: byte4[5:4] as value[5:4] and byte3[7:4] as value[3:0] - 6 bits total
      val = (packet[4] & 0x30) << 4 | packet[3];
      if (val == 0x3ff) {
        print_value("rainsecs", -1);
      } else {
        if ((packet[4] & 0x40) == 0) val >>= 4; // packet[4] bit 6: strong == 0, light == 1
        print_value("rainsecs", val);
      }
      break;

    case VP2P_TEMP:
      if (packet[3] == 0xff) {
        print_value("temp", -100);
      } else {
        val = (int)packet[3] << 4 | packet[4] >> 4;
        print_value("temp", (float)(val / 10.0));
      }
      break;

    case VP2P_HUMIDITY:
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
      print_value("rh", (float)val);
      break;

    case VP2P_WINDGUST:
      print_value("windgust", packet[3]);
      if (packet[3] != 0) {
        print_value("gustref", packet[5] & 0xf0 >> 4);
      }
      break;

    case VP2P_SOIL_LEAF:
      // currently not processed but algorithm is known
      // see https://github.com/matthewwall/weewx-meteostick/blob/master/bin/user/meteostick.py
      print_value("soilleaf", -1);
      break;

    case VUEP_VCAP:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      print_value("vcap", (float)(val / 300.0));
      break;

    case VUEP_VSOLAR:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      print_value("vsolar", (float)(val / 300.0));
  }

  print_value("fei", round(rd->fei * RF69_FSTEP / 1000));
  print_value("delta", rd->delta);

  Serial.println();
}

void printHex(volatile byte* packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
    Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print('-');
  }
}
