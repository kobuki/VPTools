#ifndef _PACKETFIFO_H
#define _PACKETFIFO_H

#include <Arduino.h>

#define FIFO_SIZE 8
#define PACKET_LEN 10

struct __attribute__((packed)) RadioData {
  byte packet[PACKET_LEN];
  byte channel;
  byte rssi;
  int16_t fei;
  uint32_t delta;
};

class PacketFifo {
  public:
    bool queue(byte* packet, byte channel, byte rssi, int16_t fei, uint32_t delta);
    RadioData* dequeue();
    void flush();
    bool hasElements();

  private:
    RadioData packetFifo[FIFO_SIZE];
    byte packetIn, packetOut, qLen;
};

#endif
