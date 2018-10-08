// Sample usage of the DavisRFM69 library to sniff the packets from a Davis Instruments
// wireless Integrated Sensor Suite (ISS), demostrating compatibility between the RFM69
// and the TI CC1020 transmitter used in that hardware.  Packets received from the ISS are
// passes through to the serial port.  This code retains some of the debugging
// functionality of the LowPowerLabs Gateway sketch, on which this code is based.
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014-2016 dekaymail@gmail.com
// Example released under the MIT License (http://opensource.org/licenses/mit-license.php)
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
//
// This program has been tested on an STM32F103C8 module with an 
// attached RFM69W transceiver module connected as follows:
//      RFM69W      STM32F103C8
//      MISO        A6
//      MOSI        A7
//      SCK         A5
//      CS/SS       A4
//      DIO0        A0 (Interrupt 0)
//
// Do NOT connect the Reset of the two together!!! Reset on the STM32F1 is active LOW and on
// the RFM69 it is active HIGH.
// 
//  See also http://wiki.stm32duino.com/index.php?title=Blue_Pill

#include "DavisRFM69.h"
#include <SPI.h>

// NOTE: *** One of DAVIS_FREQS_US, DAVIS_FREQS_EU, DAVIS_FREQS_AU, or
// DAVIS_FREQS_NZ MUST be defined at the top of DavisRFM69.h ***

//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define SERIAL_BAUD   115200
#define PACKET_INTERVAL 2555
#define DEBUG true
boolean strmon = false;       // Print the packet when received?

DavisRFM69 radio;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize();
  radio.setChannel(0);              // Frequency / Channel is *not* set in the initialization. Do it right after.
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  Serial.println(F("Waiting for signal in region defined in DavisRFM69.h"));
}

unsigned long lastRxTime = 0;
byte hopCount = 0;

boolean goodCrc = false;
int16_t goodRssi = -999;

void loop() {
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == 'r') //r=dump all register values
    {
      radio.readAllRegs();
      Serial.println();
    }
    if (input == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
      Serial.print(F("Radio Temp is "));
      Serial.print(temperature);
      Serial.print(F("C, "));
      Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
      Serial.println(F("F"));
    }
  }

  // The check for a zero CRC value indicates a bigger problem that will need
  // fixing, but it needs to stay until the fix is in.
  // TODO Reset the packet statistics at midnight once I get my clock module.
  if (radio.receiveDone()) {
    packetStats.packetsReceived++;
    unsigned int crc = radio.crc16_ccitt(radio.DATA, 6);
    if ((crc == (word(radio.DATA[6], radio.DATA[7]))) && (crc != 0)) {
      // This is a good packet
      goodCrc = true;
      goodRssi = radio.RSSI;
      packetStats.receivedStreak++;
      hopCount = 1;
    } else {
      goodCrc = false;
      packetStats.crcErrors++;
      packetStats.receivedStreak = 0;
    }

    if (strmon) printStrm();
#if DEBUG
    // Debugging stuff
    Serial.print(millis());
    Serial.print(F(":  "));
    Serial.print(radio.CHANNEL);
    Serial.print(F(" - Data: "));
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
      Serial.print(radio.DATA[i], HEX);
      Serial.print(F(" "));
    }
    Serial.print(F("  RSSI: "));
    Serial.println(radio.RSSI);
    int freqError = radio.readReg(0x21) << 8 |radio.readReg(0x22);
    Serial.print(F("      Freq error): "));
    Serial.println(freqError);
#endif
    // If packet was received earlier than expected, that was probably junk. Don't hop.
    // I use a simple heuristic for this.  If the CRC is bad and the received RSSI is
    // a lot less than the last known good RSSI, then don't hop.
    if (goodCrc && (radio.RSSI < (goodRssi + 15))) {
      lastRxTime = millis();
      radio.hop();
#if DEBUG
      Serial.print(millis());
      Serial.println(F(":  Hopped channel and ready to receive."));
#endif
    } else {
      radio.waitHere();
#if DEBUG
      Serial.print(millis());
      Serial.println(F(":  Waiting here"));
#endif
    }
  }

  // If a packet was not received at the expected time, hop the radio anyway
  // in an attempt to keep up.  Give up after 4 failed attempts.  Keep track
  // of packet stats as we go.  I consider a consecutive string of missed
  // packets to be a single resync.  Thx to Kobuki for this algorithm.
  if ((hopCount > 0) && ((millis() - lastRxTime) > (hopCount * PACKET_INTERVAL + 200))) {
    packetStats.packetsMissed++;
    if (hopCount == 1) packetStats.numResyncs++;
    if (++hopCount > 4) hopCount = 0;
    radio.hop();
#if DEBUG
    Serial.print(millis());
    Serial.println(F(":  Hopped channel and ready to receive."));
#endif
  }
}

void printStrm() {
  for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
    Serial.print(i);
    Serial.print(F(" = "));
    Serial.print(radio.DATA[i], HEX);
    Serial.print(F("\n\r"));
  }
  Serial.print(F("\n\r"));
}
