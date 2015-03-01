
#include <SPI.h>
#include <DavisRFM69.h>
#include <TimerOne.h>
#include <PacketFifo.h>

#define LED           9  // Moteinos have LEDs on D9
#define SERIAL_BAUD   115200

#define RESYNC_THRESHOLD 50       // max. number of lost packets from a station before a full rediscovery
#define LATE_PACKET_THRESH 5000   // packet is considered missing after this many micros
#define POST_RX_WAIT 2000         // RX "settle" delay

DavisRFM69 radio;

char hs[24];
char* hex = "0123456789abcdef";

PacketFifo fifo;
volatile unsigned long packets, lostPackets, numResyncs;
volatile byte stationsFound = 0;
volatile byte curStation = 0;

#define NUM_STATIONS 3

// id, type, active
Station stations[NUM_STATIONS] = {
  { 0, STYPE_ISS,         true },
  { 1, STYPE_WLESS_ANEMO, true },
  { 2, STYPE_TEMP_HUM,    true }
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize();
  radio.setChannel(0);
  fifo.flush();
  initStations();
  Timer1.initialize(1000); // periodical interrupts every ms for missed packet detection
  Timer1.attachInterrupt(handleTimerInt, 0);
  radio.setUserInterrupt(handleRadioInt);
}

char f = 0;

void loop() {
  if (fifo.hasElements()) {
    decode_packet(fifo.dequeue());
  }
}

// Handle missed packets. Called from Timer1 ISR every ms
void handleTimerInt() {

  unsigned long lastRx = micros();
  bool readjust = false;
  
  // find and adjust 'last seen' rx time on all stations with older reception timestamp than their period + threshold
  // that is, find missed packets
  for (byte i = 0; i < NUM_STATIONS; i++) {
    if (stations[i].interval > 0 && (lastRx - stations[i].lastRx) > stations[i].interval + LATE_PACKET_THRESH) {
      if (stationsFound == NUM_STATIONS && stations[curStation].active) lostPackets++;
      stations[i].lostPackets++;
      if (stations[i].lostPackets > RESYNC_THRESHOLD) {
        numResyncs++;
        stationsFound = 0;
        initStations();
        return;
      } else {
        stations[i].lastRx += stations[i].interval; // when packet should have been received
        stations[i].channel = nextChannel(stations[i].channel); // skip station's next channel in hop seq
        readjust = true;
      }
    }
  }

  // find/set earliest expected rx channel or in Phase 1, reset radio on current channel
  if (readjust) {
    nextStation();
    if (stationsFound < NUM_STATIONS) {
      radio.setChannel(radio.CHANNEL);
    } else {
      radio.setChannel(stations[curStation].channel);
    }
  }

}

// Handle received packets, called from RFM69 ISR
void handleRadioInt() {

  unsigned long lastRx = micros();

  unsigned int calcCrc = radio.crc16_ccitt(radio.DATA, 6);  // calculated and
  unsigned int rxCrc = word(radio.DATA[6], radio.DATA[7]);  // received crc
  
  f = 1;
  delayMicroseconds(POST_RX_WAIT); // we need this, no idea why, but makes reception almost perfect
                                   // probably needed by the module to settle something after RX

  //fifo.queue((byte*)radio.DATA, radio.CHANNEL, -radio.RSSI);

  // packet passed crc?
  if (calcCrc == rxCrc && rxCrc != 0) {

    // station id is byte 0:0-2
    byte id = radio.DATA[0] & 7;
    int stIx = findStation(id);
    
    // if we have no station cofigured for this id (at all; can still be be !active), ignore the packet
    if (stIx < 0) {
      radio.setChannel(radio.CHANNEL);
      return;
    }
    
    // Phase 1: station discovery
    // stay on a fixed channel and store last rx timestamp of discovered stations in station array
    // interval is used as 'station seen' flag
    if (stationsFound < NUM_STATIONS && stations[stIx].interval == 0) {

      stations[stIx].channel = nextChannel(radio.CHANNEL);
      stations[stIx].lastRx = lastRx;
      stations[stIx].lostPackets = 0;
      stations[stIx].interval = (41 + id) * 1000000 / 16; // Davis' official tx interval in us
      stationsFound++;
      nextStation(); // skip to next station expected to tx

      // reset current radio channel in Phase 1 or start Phase 2
      if (stationsFound < NUM_STATIONS) {
        radio.setChannel(radio.CHANNEL);
      } else {
        radio.setChannel(stations[curStation].channel);
      }
      
      return;

    } else {
      
      // Phase 2: normal reception
      // 8 received packet bytes including received CRC, the channel and RSSI go to the fifo
      if (stationsFound == NUM_STATIONS && stations[curStation].active) {
        packets++;
        fifo.queue((byte*)radio.DATA, radio.CHANNEL, -radio.RSSI);
      }

      // successful reception - skip to next/earliest expected station
      stations[curStation].lostPackets = 0;
      stations[curStation].lastRx = lastRx;
      stations[curStation].channel = nextChannel(radio.CHANNEL);
      nextStation();
      if (stationsFound < NUM_STATIONS) { // Phase 1/2 check as usual
        radio.setChannel(radio.CHANNEL);
      } else {
        radio.setChannel(stations[curStation].channel);
      }

      return;
    }

  } else {
    radio.setChannel(radio.CHANNEL); // this always has to be done somewhere right after reception, even for ignored/bogus packets
  }

}

// Calculate the next hop of the specified channel
byte nextChannel(byte channel) {
  return ++channel % DAVIS_FREQ_TABLE_LENGTH;
}

// Find the station index in stations[] for station expected to tx the earliest and update curStation
void nextStation() {
  unsigned long earliest = 0xffffffff;
  unsigned long now = micros();
  for (int i = 0; i < NUM_STATIONS; i++) {
    unsigned long current = stations[i].lastRx + stations[i].interval - now;
    if (stations[i].interval > 0 && current < earliest) {
      earliest = current;
      curStation = i;
    }
  }
}

// Find station index in stations[] for a station ID (-1 if doesn't exist)
int findStation(byte id) {
  for (byte i = 0; i < NUM_STATIONS; i++) {
    if (stations[i].id == id) return i;
  }
  return -1;
}

// Reset station array to safe defaults
void initStations() {
  for (byte i = 0; i < NUM_STATIONS; i++) {
    stations[i].channel = 0;
    stations[i].lastRx = 0;
    stations[i].interval = 0;
    stations[i].lostPackets = 0;
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
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(F(" "));
}

void print_value(char* vname, int value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(F(" "));
}

void print_value(char* vname, float value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value, 1); Serial.print(F(" "));
}

void print_value(char* vname, long value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(F(" "));
}

void print_value(char* vname, unsigned long value) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(F(" "));
}

void decode_packet(RadioData* rd) {
  int val;
  byte* packet = rd->packet;

  packetToHex(packet, 6);
  print_value("raw", hs);
  print_value("station", packet[0] & 0x7);
  print_value("packets", packets);
  print_value("lostp", lostPackets);
  print_value("ratio", (float)(packets * 100.0 / (packets + lostPackets)));
  print_value("channel", rd->channel);
  print_value("rssi", -rd->rssi);

  print_value("batt", (char*)(packet[0] & 0x8 ? "err" : "ok"));
  print_value("windv", packet[1]);
  print_value("windd", packet[2]);

  byte id = radio.DATA[0] & 7;
  int stIx = findStation(id);
  byte byte4MSN;

  if (stations[stIx].type == STYPE_ISS)

    switch (packet[0] >> 4) {

      case VP2P_UV:
        if (packet[3] == 0xff) {
          print_value("uv", -1);
        } else {
          val = packet[3] << 8 | packet[4] >> 6;
          print_value("uv", (float)(val / 50.0));
        }
        break;

      case VP2P_RAINSECS:
        byte4MSN = packet[4] >> 4;
        if (packet[3] == 0xff) {
          print_value("rainsecs", -1);
        } else {
          if (byte4MSN < 4) {
            val = (packet[3] >> 4) + packet[4];
          } else {
            val = packet[3] + (byte4MSN - 4) * 262;
          }
          print_value("rainsecs", val);
        }
        break;

      case VP2P_SOLAR:
        if (packet[3] == 0xff) {
          print_value("solar", -1);
        } else {
          val = packet[3] << 8 | packet[4] >> 6;
          print_value("solar", (float)(val * 1.757936));
        }
        break;

      case VP2P_TEMP:
        val = (int)packet[3] << 4 | packet[4] >> 4;
        print_value("temp", (float)(val / 10.0));
        break;

      case VP2P_HUMIDITY:
        val = ((packet[4] >> 4) << 8 | packet[3]) / 10;
        print_value("rh", (float)val);
        break;

      case VP2P_RAIN:
        print_value("rain", packet[3]);
    }
  
  if (stations[stIx].type == STYPE_ISS || stations[stIx].type == STYPE_WLESS_ANEMO) {
    switch (packet[0] >> 4) {
      case VP2P_WINDGUST:
        print_value("windgust", packet[3]);
    }
  }

  Serial.println();
}

void packetToHex(volatile byte* packet, byte len) {
  hs[len * 3 - 1] = 0;
  int x = 0;
  for (byte i = 0; i < len; i++) {
    hs[x++] = hex[packet[i] >> 4];
    hs[x++] = hex[packet[i] & 0x0f];
    if (i < len - 1) hs[x++] = '-';
  }
}

