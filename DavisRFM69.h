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

// Uncomment ONE AND ONLY ONE of the four #define's below.  This determines the
// frequency table the code will use.  Note that only the US (actually North
// America) and EU frequencies are defined at this time.  Australia and New
// Zealand are placeholders.  Note however that the frequencies for AU and NZ
// are not known at this time.
//#define DAVIS_FREQS_US
#define DAVIS_FREQS_EU
//#define DAVIS_FREQS_AU
//#define DAVIS_FREQS_NZ

#include <Davisdef.h>
#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater

#define DAVIS_PACKET_LEN      8 // ISS has fixed packet lengths of eight bytes, including CRC
#define SPI_CS               SS // SS is the SPI slave select pin, for instance D10 on atmega328
#define RF69_IRQ_PIN          2 // INT0 on AVRs should be connected to DIO0 (ex on Atmega328 it's D2)
#define CSMA_LIMIT          -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP       0 // XTAL OFF
#define RF69_MODE_STANDBY     1 // XTAL ON
#define RF69_MODE_SYNTH       2 // PLL ON
#define RF69_MODE_RX          3 // RX MODE
#define RF69_MODE_TX          4 // TX MODE

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value

class DavisRFM69 {
  public:
    static volatile byte DATA[DAVIS_PACKET_LEN];  // recv/xmit buf, including header, CRC, and RSSI value
    static volatile byte _mode; //should be protected?
    static volatile bool _packetReceived;
    static volatile byte CHANNEL;
    static volatile int RSSI;

    DavisRFM69(byte slaveSelectPin=SPI_CS, byte interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false) {
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _mode = RF69_MODE_STANDBY;
      _packetReceived = false;
      _powerLevel = 31;
      _isRFM69HW = isRFM69HW;
    }

    void send(byte toAddress, const void* buffer, byte bufferSize, bool requestACK=false);
    static volatile byte hopIndex;
    void setChannel(byte channel);
    void hop();
    unsigned int crc16_ccitt(volatile byte *buf, byte len, unsigned int initCrc = 0);

    void initialize();
    bool canSend();
    void send(const void* buffer, byte bufferSize);
    bool receiveDone();
    void setFrequency(uint32_t FRF);
    void setCS(byte newSPISlaveSelect);
    int readRSSI(bool forceTrigger=false);
    void setHighPower(bool onOFF=true); //have to call it after initialize for RFM69HW
    void setPowerLevel(byte level); //reduce/increase transmit power level
    void sleep();
    byte readTemperature(byte calFactor=0); //get CMOS temperature (8bit)
    void rcCalibration(); //calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    byte readReg(byte addr);
    void writeReg(byte addr, byte val);
    void readAllRegs();
    void setTxMode(bool txMode);
    void setUserInterrupt(void (*function)());

  protected:
    static volatile bool txMode;
    void (*userInterrupt)();

    void virtual interruptHandler();
    void sendFrame(const void* buffer, byte size);
    byte reverseBits(byte b);

    static void isr0();

    static DavisRFM69* selfPointer;
    byte _slaveSelectPin;
    byte _interruptPin;
    byte _powerLevel;
    bool _isRFM69HW;

    void receiveBegin();
    void setMode(byte mode);
    void setHighPowerRegs(bool onOff);
    void select();
    void unselect();
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
#elif defined (DAVIS_FREQS_EU)
#warning ** USING EUROPEAN FREQUENCY TABLE **
#define DAVIS_FREQ_TABLE_LENGTH 5
static const uint8_t __attribute__ ((progmem)) FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
{
  {0xD9, 0x04, 0x45},
  {0xD9, 0x13, 0x04},
  {0xD9, 0x21, 0xC2},
  {0xD9, 0x0B, 0xA4},
  {0xD9, 0x1A, 0x63}
};
#elif defined (DAVIS_FREQS_AU)
#error ** ERROR DAVIS FREQS FOR AU ARE NOT KNOWN AT THIS TIME. ONLY US & EU DEFINED **
#elif defined (DAVIS_FREQS_NZ)
#error ** ERROR DAVIS FREQS FOR NZ ARE NOT KNOWN AT THIS TIME. ONLY US & EU DEFINED **
#else
#error ** ERROR DAVIS_FREQS MUST BE DEFINED AS ONE OF _US, _EU, _AZ, or NZ **
#endif  // DAVIS_FREQS

// For the packet stats structure used in response to the RXCHECK command

struct __attribute__((packed)) PacketStats
{
  uint16_t packetsReceived;
  uint16_t packetsMissed;
  uint16_t numResyncs;
  uint16_t receivedStreak;
  uint16_t crcErrors;
};

static PacketStats packetStats = {0, 0, 0, 0 ,0};

#define LOOP_PACKET_LENGTH 97

// LOOP data packet length and structure.
// See http://www.davisnet.com/support/weather/download/VantageSerialProtocolDocs_v230.pdf
// The CRC is not included in the here but calculated on the fly.
// WARNING: Many of the items below are not implemented!!!
struct __attribute__((packed)) LoopPacket
{
  char loo[3];          // "LOO" at packet start indicates Rev B packet type
  byte barTrend;        // Barometric trend
  byte packetType;      // Packet type, always zero
  uint16_t nextRecord;  // Location in archive memory where next packet will be written
  uint16_t barometer;   // Current barometer in Hg / 1000
  int16_t insideTemperature;  // Inside temperature in tenths of degrees
  byte insideHumidity;        // Inside relative humidity in percent
  int16_t outsideTemperature; // Outside temperature in tenths of degrees
  byte windSpeed;             // Wind speed in miles per hour
  byte tenMinAvgWindSpeed;    // Average wind speed over last ten minutes
  uint16_t windDirection;     // Wind direction from 1 to 360 degrees (0 = no wind data)
  byte extraTemperatures[7];  // Temps from seven extra stations in whole degrees F offset by 90
  byte soilTemperatures[4];   // Soil temps from four extra sensors.  Format as above.
  byte leafTemperatures[4];   // Leaf temps from four extra sensors.  Format as above.
  byte outsideHumidity;       // Outside relative humidity in %.
  byte extraHumidities[7];    // Relative humidity in % from seven extra stations.
  uint16_t rainRate;          // Rain rate as number of rain clicks per hour (e.g 256 = 2.56 in/hr)
  byte uV;                    // UV index
  uint16_t solarRadiation;    // Solar radiation in Watts/m^2
  uint16_t stormRain;         // Storm rain stored as hundredths of an inch
  uint16_t startDateOfStorm;  // Bits 15-12 is month, bits 11-7 is day, and bits 6-0 is year offset by 2000
  uint16_t dayRain;           // Rain today sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t monthRain;         // Rain this month sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t yearRain;          // Rain this year sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t dayET;             // ET today sent as thousandths of an inch
  uint16_t monthET;           // ET this month sent as hundredths of an inch
  uint16_t yearET;            // ET this year sent as hundredths of an inch
  byte soilMoistures[4];      // Soil moisture in centibars from up to four soil sensors
  byte leafWetnesses[4];      // A scale number from 0-15. 0 means very dry, 15 very wet. Supports four leaf sensors.
  byte insideAlarms;          // Currently active inside alarms.
  byte rainAlarms;            // Currently active rain alarms.
  uint16_t outsideAlarms;     // Currently active outside alarms.
  byte extraTempHumAlarms[8]; // Currently active temperature / humidity alarms for up to eight stations.
  byte soilLeafAlarms[4];     // Currently active soil and leaf alarms for up to four sensors
  byte transmitterBatteryStatus;  // Transmitter battery status (0 or 1)
  uint16_t consoleBatteryVoltage; // Console voltage as  ((Data * 300)/512)/100.0
  byte forecastIcons;             // Forecast icons
  byte forecastRuleNumber;        // Forecast rule number
  uint16_t timeOfSunrise;         // Sunrise time stored as hour * 100 + min.
  uint16_t timeOfSunset;          // Sunset time stored as hour * 100 + min.
  char lfcr[2];                   // Carriage return / line feed
};

// Initializer for the LOOP packet.  Static const so it goes into flash.
static const LoopPacket loopInit =
{
  {'L', 'O', 'O'}, // Identifies this as a Rev B packet
  255,             // Signal barometric trend as not known
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  0,
  {0, 0, 0, 0, 0, 0, 0},
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  0,
  0,
  0,
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0},
  0,
  0,
  0,
  0,
  0,
  0,
  {'\n', '\r'}  // End of packet line feed and carriage return
};

// archive record type for DMP and DMPAFT commands
struct __attribute__((packed)) ArchiveRec
{
  uint16_t dateStamp;
  uint16_t timeStamp;
  int16_t outsideTemp;
  int16_t highOutTemp;
  int16_t lowOutTemp;
  uint16_t rainfall;
  uint16_t highRainRate;
  uint16_t barometer;
  uint16_t solarRad;
  uint16_t windSamples;
  int16_t insideTemp;
  byte    insideHum;
  byte    outsideHum;
  byte    avgWindSpd;
  byte    highWindSpd;
  byte    dirHiWindSpd;
  byte    prevWindDir;
  byte    avgUVIndex;
  byte    eT;
  int16_t highSolarRad;
  byte    highUVIdx;
  byte    forecastRule;
  byte    leafTemp0;
  byte    leafTemp1;
  byte    leafWet0;
  byte    leafWet1;
  byte    soilTemp0;
  byte    soilTemp1;
  byte    soilTemp2;
  byte    soilTemp3;
  byte    recType;
  byte    extraHum0;
  byte    extraHum1;
  byte    extraTemp0;
  byte    extraTemp1;
  byte    extraTemp2;
  byte    soilMoist0;
  byte    soilMoist1;
  byte    soilMoist2;
  byte    soilMoist3;
};

// fake archive record (temporary)
static const ArchiveRec fakeArchiveRec =
{
  26 + 2 * 32 + (2014 - 2000) * 512,
  (19 * 100) + 25,
  32767,
  -32768,
  32767,
  0,
  0,
  0,
  32767,
  0,
  32767,
  255,
  255,
  255,
  0,
  255,
  255,
  255,
  0,
  0,
  0,
  193,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  0,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255
};

#endif  // DAVISRFM_h
