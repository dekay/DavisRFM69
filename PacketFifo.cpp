
#include "packetfifo.h"

bool PacketFifo::queue(volatile byte* p, byte rssi) {
  if (qLen < FIFO_SIZE) {
    for (byte i = 0; i < RSSI_OFFSET; i++) packetFifo[packetIn][i] = p[i];
    packetFifo[packetIn][RSSI_OFFSET] = rssi;
    if (++packetIn == FIFO_SIZE) packetIn = 0;
    qLen++;
    return true;
  } else {
    return false;
  }
}

volatile byte* PacketFifo::dequeue() {
  if (qLen > 0) {
    qLen--;
    volatile byte* rv = packetFifo[packetOut];
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