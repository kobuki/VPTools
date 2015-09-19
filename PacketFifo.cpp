
#include "packetfifo.h"

bool PacketFifo::queue(byte* packet, byte channel, byte rssi, int16_t fei, uint32_t delta) {
  if (qLen < FIFO_SIZE) {
    memcpy(&packetFifo[packetIn].packet, packet, PACKET_LEN);
    packetFifo[packetIn].channel = channel;
    packetFifo[packetIn].rssi = rssi;
    packetFifo[packetIn].fei = fei;
    packetFifo[packetIn].delta = delta;
    if (++packetIn == FIFO_SIZE) packetIn = 0;
    qLen++;
    return true;
  } else {
    return false;
  }
}

RadioData* PacketFifo::dequeue() {
  if (qLen > 0) {
    qLen--;
    RadioData* rv = &packetFifo[packetOut];
    if (++packetOut == FIFO_SIZE) packetOut = 0;
    return rv;
  } else {
    return NULL;
  }
}

void PacketFifo::flush() {
  packetIn = packetOut = qLen = 0;
}

bool PacketFifo::hasElements() {
  return qLen > 0;
}