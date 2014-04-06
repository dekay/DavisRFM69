#ifndef _PACKETFIFO_H
#define _PACKETFIFO_H

#include <Arduino.h>

#define FIFO_SIZE 8
#define DATA_SIZE 9 // 8 bytes Davis radio packet with CRC and 1 byte RSSI
#define RSSI_OFFSET 8

class PacketFifo {
public:
    bool queue(volatile byte* p, byte rssi);
    volatile byte* dequeue();
    void flush();
    bool hasElements();

private:
    volatile byte packetFifo[FIFO_SIZE][DATA_SIZE];
    byte packetIn, packetOut, qLen;
};

#endif
