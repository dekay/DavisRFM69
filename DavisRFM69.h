// Driver definition for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/

#ifndef DAVISRFM69_h
#define DAVISRFM69_h
#include <Davisdef.h>
#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater
#include <../RFM69/RFM69.h>

class DavisRFM69: public RFM69 {
  public:
    void send(byte toAddress, const void* buffer, byte bufferSize, bool requestACK=false);
    static volatile byte hopIndex;
    bool initialize(byte freqBand, byte ID, byte networkID=1);
    void setChannel(byte channel);
    void hop();
    unsigned int crc16_ccitt(volatile byte *buf, byte len);

  protected:
    void virtual interruptHandler();
    void sendFrame(byte toAddress, const void* buffer, byte size, bool requestACK=false, bool sendACK=false);
    byte reverseBits(byte b);
};

// FRF_MSB, FRF_MID, and FRF_LSB for the 51 North American channels & 5 European channels.
// used by Davis in frequency hopping

#ifdef DAVIS_FREQS_US
#warning ** USING NORTH AMERICAN FREQUENCY TABLE **
#define DAVIS_FREQ_TABLE_LENGTH 51
static const uint8_t __attribute__ ((progmem)) FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
{
  {0xE3, 0xDA, 0x7C},
  {0xE1, 0x98, 0x71},
  {0xE3, 0xFA, 0x92},
  {0xE6, 0xBD, 0x01},
  {0xE4, 0xBB, 0x4D},
  {0xE2, 0x99, 0x56},
  {0xE7, 0x7D, 0xBC},
  {0xE5, 0x9C, 0x0E},
  {0xE3, 0x39, 0xE6},
  {0xE6, 0x1C, 0x81},
  {0xE4, 0x5A, 0xE8},
  {0xE1, 0xF8, 0xD6},
  {0xE5, 0x3B, 0xBF},
  {0xE7, 0x1D, 0x5F},
  {0xE3, 0x9A, 0x3C},
  {0xE2, 0x39, 0x00},
  {0xE4, 0xFB, 0x77},
  {0xE6, 0x5C, 0xB2},
  {0xE2, 0xD9, 0x90},
  {0xE7, 0xBD, 0xEE},
  {0xE4, 0x3A, 0xD2},
  {0xE1, 0xD8, 0xAA},
  {0xE5, 0x5B, 0xCD},
  {0xE6, 0xDD, 0x34},
  {0xE3, 0x5A, 0x0A},
  {0xE7, 0x9D, 0xD9},
  {0xE2, 0x79, 0x41},
  {0xE4, 0x9B, 0x28},
  {0xE5, 0xDC, 0x40},
  {0xE7, 0x3D, 0x74},
  {0xE1, 0xB8, 0x9C},
  {0xE3, 0xBA, 0x60},
  {0xE6, 0x7C, 0xC8},
  {0xE4, 0xDB, 0x62},
  {0xE2, 0xB9, 0x7A},
  {0xE5, 0x7B, 0xE2},
  {0xE7, 0xDE, 0x12},
  {0xE6, 0x3C, 0x9D},
  {0xE3, 0x19, 0xC9},
  {0xE4, 0x1A, 0xB6},
  {0xE5, 0xBC, 0x2B},
  {0xE2, 0x18, 0xEB},
  {0xE6, 0xFD, 0x42},
  {0xE5, 0x1B, 0xA3},
  {0xE3, 0x7A, 0x2E},
  {0xE5, 0xFC, 0x64},
  {0xE2, 0x59, 0x16},
  {0xE6, 0x9C, 0xEC},
  {0xE2, 0xF9, 0xAC},
  {0xE4, 0x7B, 0x0C},
  {0xE7, 0x5D, 0x98}
}; 
#else
#ifdef DAVIS_FREQS_EU
#warning ** USING EUROPEAN FREQUENCY TABLE **
#define DAVIS_FREQ_TABLE_LENGTH 5
static const uint8_t __attribute__ ((progmem)) FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
{
  {0xD9, 0x4, 0x45},
  {0xD9, 0x13, 0x4},
  {0xD9, 0x21, 0xC2},
  {0xD9, 0xB, 0xA4},
  {0xD9, 0x1A, 0x63}
};
#else
#ifdef DAVIS_FREQS_AU
#error ** ERROR DAVIS FREQS FOR AU ARE NOT KNOWN AT THIS TIME. ONLY US & EU DEFINED **
#else
#ifdef DAVIS_FREQS_NZ
#error ** ERROR DAVIS FREQS FOR NZ ARE NOT KNOWN AT THIS TIME. ONLY US & EU DEFINED **
#else
#error ** ERROR DAVIS_FREQS MUST BE DEFINED AS ONE OF _US, _EU, _AZ, or NZ **
#endif  // DAVIS_FREQS_US
#endif  // DAVIS_FREQS_EU
#endif  // DAVIS_FREQS_AU
#endif  // DAVIS_FREQS_NZ

#endif  // DAVISRFM_h
