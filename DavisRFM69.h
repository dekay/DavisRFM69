// Driver definition for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS).  This library has been tested against both the
// Moteino from LowPowerLab, and an ESP-12E wired directly to an RFM69W module.
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014-2016 dekaymail@gmail.com
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
//#define DAVIS_FREQS_EU
//#define DAVIS_FREQS_AU
//#define DAVIS_FREQS_NZ

#include <Davisdef.h>
#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater

#define DAVIS_PACKET_LEN 10 // ISS has fixed packet length of 10 bytes,
                            // including CRC and retransmit CRC
#define RF69_SPI_CS  SS     // SS is the SPI slave select pin
                            // For instance D10 on ATmega328
// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN 2
  #define RF69_IRQ_NUM 0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN 2
  #define RF69_IRQ_NUM 2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN 3
  #define RF69_IRQ_NUM 0
#elif defined(ESP8266)
  // ESP interrupt number and pin number are a direct relation
  // See http://www.esp8266.com/viewtopic.php?f=32&t=4694
  #define RF69_IRQ_PIN 5
  #define RF69_IRQ_NUM 5
#elif defined(ARDUINO_ARCH_STM32F1)
  #define RF69_IRQ_PIN PA0
  #define RF69_IRQ_NUM 0
#endif

#define CSMA_LIMIT          -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP     0 // XTAL OFF
#define RF69_MODE_STANDBY   1 // XTAL ON
#define RF69_MODE_SYNTH     2 // PLL ON
#define RF69_MODE_RX        3 // RX MODE
#define RF69_MODE_TX        4 // TX MODE

// available frequency bands
#define RF69_315MHZ         31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ         43
#define RF69_868MHZ         86
#define RF69_915MHZ         91

#define null                0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value

// These values aren't in the upstream version of RF69registers.h
#define REG_TESTAFC         0x71

#define RF_FDEVMSB_4800     0x00 // Used for Davis console reception
#define RF_FDEVLSB_4800     0x4e
#define RF_FDEVMSB_9900     0x00 // Used for Davis ISS transmission
#define RF_FDEVLSB_9900     0xa1

// Davis VP2 standalone station types, defined by kobuki
#define STYPE_ISS           0x0 // ISS
#define STYPE_TEMP_ONLY     0x1 // Temperature Only Station
#define STYPE_HUM_ONLY      0x2 // Humidity Only Station
#define STYPE_TEMP_HUM      0x3 // Temperature/Humidity Station
#define STYPE_WLESS_ANEMO   0x4 // Wireless Anemometer Station
#define STYPE_RAIN          0x5 // Rain Station
#define STYPE_LEAF          0x6 // Leaf Station
#define STYPE_SOIL          0x7 // Soil Station
#define STYPE_SOIL_LEAF     0x8 // Soil/Leaf Station
#define STYPE_SENSORLINK    0x9 // SensorLink Station (not supported for the VP2)
#define STYPE_OFF           0xA // No station - OFF

// Davis packet types, also defined by kobuki
#define VP2P_UV             0x4 // UV index
#define VP2P_RAINSECS       0x5 // seconds between rain bucket tips
#define VP2P_SOLAR          0x6 // solar irradiation
#define VP2P_TEMP           0x8 // outside temperature
#define VP2P_WINDGUST       0x9 // 10-minute wind gust
#define VP2P_HUMIDITY       0xA // outside humidity
#define VP2P_RAIN           0xE // rain bucket tips counter

class DavisRFM69 {
  public:
    static volatile uint8_t DATA[DAVIS_PACKET_LEN];  // recv/xmit buf, including header, CRC
    static volatile uint8_t _mode; //should be protected?
    static volatile bool _packetReceived;
    static volatile uint8_t CHANNEL;
    static volatile int16_t RSSI;

    DavisRFM69(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM) {
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _interruptNum = interruptNum;
      _mode = RF69_MODE_STANDBY;
      _packetReceived = false;
      _powerLevel = 31;
      _isRFM69HW = isRFM69HW;
    }

    void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);
    static volatile uint8_t hopIndex;
    void setChannel(uint8_t channel);
    void hop();
    void waitHere();
    uint16_t crc16_ccitt(volatile uint8_t *buf, uint8_t len, uint16_t initCrc = 0);

    void initialize();
    bool canSend();
    void send(const void* buffer, uint8_t bufferSize);
    bool receiveDone();
    void setFrequency(uint32_t FRF);
    void setCS(uint8_t newSPISlaveSelect);
    int16_t readRSSI(bool forceTrigger=false);
    void setHighPower(bool onOFF=true); // Have to call it after initialize for RFM69HW
    void setPowerLevel(uint8_t level);  // Reduce/increase transmit power level
    void sleep();
    uint8_t readTemperature(uint8_t calFactor=0); // Get CMOS temperature (8bit)
    void rcCalibration(); //calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    void readAllRegs();

  protected:
    void virtual interruptHandler();
    void sendFrame(const void* buffer, uint8_t size);
    uint8_t reverseBits(uint8_t b);

    static void isr0();

    static DavisRFM69* selfPointer;
    uint8_t _slaveSelectPin;
    uint8_t _interruptPin;
    uint8_t _interruptNum;
    uint8_t _powerLevel;
    bool _isRFM69HW;
    uint8_t _SPCR;
    uint8_t _SPSR;

    void receiveBegin();
    void setMode(uint8_t mode);
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
#warning ** USING AUSTRALIAN FREQUENCY TABLE **
// Australian frequencies courtesy of kriptic.  Thanks!!!
// http://www.wxforum.net/index.php?topic=25980.msg250371#msg250371
#define DAVIS_FREQ_TABLE_LENGTH 51
static const uint8_t __attribute__ ((progmem)) FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
{
   {0xE5, 0x84, 0xDD},
   {0xE6, 0x43, 0x43},
   {0xE7, 0x1F, 0xCE},
   {0xE6, 0x7F, 0x7C},
   {0xE5, 0xD5, 0x0E},
   {0xE7, 0x5B, 0xF7},
   {0xE6, 0xC5, 0x81},
   {0xE6, 0x07, 0x2B},
   {0xE6, 0xED, 0xA1},
   {0xE6, 0x61, 0x58},
   {0xE5, 0xA3, 0x02},
   {0xE6, 0xA7, 0x8D},
   {0xE7, 0x3D, 0xB2},
   {0xE6, 0x25, 0x3F},
   {0xE5, 0xB7, 0x0A},
   {0xE6, 0x93, 0x85},
   {0xE7, 0x01, 0xDB},
   {0xE5, 0xE9, 0x26},
   {0xE7, 0x70, 0x00},
   {0xE6, 0x57, 0x6C},
   {0xE5, 0x98, 0xF5},
   {0xE6, 0xB1, 0x99},
   {0xE7, 0x29, 0xDB},
   {0xE6, 0x11, 0x37},
   {0xE7, 0x65, 0xE3},
   {0xE5, 0xCB, 0x33},
   {0xE6, 0x75, 0x60},
   {0xE6, 0xD9, 0xA9},
   {0xE7, 0x47, 0xDF},
   {0xE5, 0x8E, 0xF9},
   {0xE6, 0x2F, 0x4B},
   {0xE7, 0x0B, 0xB6},
   {0xE6, 0x89, 0x68},
   {0xE5, 0xDF, 0x2B},
   {0xE6, 0xBB, 0xA5},
   {0xE7, 0x79, 0xFB},
   {0xE6, 0xF7, 0xAE},
   {0xE5, 0xFD, 0x2F},
   {0xE6, 0x4D, 0x4F},
   {0xE6, 0xCF, 0x8D},
   {0xE5, 0xAD, 0x0E},
   {0xE7, 0x33, 0xD7},
   {0xE6, 0x9D, 0x91},
   {0xE6, 0x1B, 0x33},
   {0xE6, 0xE3, 0xA5},
   {0xE5, 0xC1, 0x16},
   {0xE7, 0x15, 0xC2},
   {0xE5, 0xF3, 0x33},
   {0xE6, 0x6B, 0x64},
   {0xE7, 0x51, 0xDB},
   {0xE6, 0x39, 0x58}
};
#elif defined (DAVIS_FREQS_NZ)
#error ** ERROR DAVIS FREQS FOR NZ ARE NOT KNOWN AT THIS TIME. **
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
  char      loo[3];      // "LOO" at packet start indicates Rev B packet type
  uint8_t   barTrend;    // Barometric trend
  uint8_t   packetType;  // Packet type, always zero
  uint16_t  nextRecord;  // Location in archive memory where next packet will be written
  uint16_t  barometer;   // Current barometer in Hg / 1000
  int16_t   insideTemperature;  // Inside temperature in tenths of degrees
  uint8_t   insideHumidity;     // Inside relative humidity in percent
  int16_t   outsideTemperature; // Outside temperature in tenths of degrees
  uint8_t   windSpeed;          // Wind speed in miles per hour
  uint8_t   tenMinAvgWindSpeed; // Average wind speed over last ten minutes
  uint16_t  windDirection;      // Wind direction from 1 to 360 degrees (0 = no wind data)
  uint8_t   extraTemperatures[7];  // Temps from seven extra stations in whole degrees F offset by 90
  uint8_t   soilTemperatures[4];   // Soil temps from four extra sensors.  Format as above.
  uint8_t   leafTemperatures[4];   // Leaf temps from four extra sensors.  Format as above.
  uint8_t   outsideHumidity;       // Outside relative humidity in %.
  uint8_t   extraHumidities[7];    // Relative humidity in % from seven extra stations.
  uint16_t  rainRate;          // Rain rate as number of rain clicks per hour (e.g 256 = 2.56 in/hr)
  uint8_t   uV;                // UV index
  uint16_t  solarRadiation;    // Solar radiation in Watts/m^2
  uint16_t  stormRain;         // Storm rain stored as hundredths of an inch
  uint16_t  startDateOfStorm;  // Bits 15-12 is month, bits 11-7 is day, and bits 6-0 is year offset by 2000
  uint16_t  dayRain;           // Rain today sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t  monthRain;         // Rain this month sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t  yearRain;          // Rain this year sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t  dayET;             // ET today sent as thousandths of an inch
  uint16_t  monthET;           // ET this month sent as hundredths of an inch
  uint16_t  yearET;            // ET this year sent as hundredths of an inch
  uint8_t   soilMoistures[4];  // Soil moisture in centibars from up to four soil sensors
  uint8_t   leafWetnesses[4];  // A scale number from 0-15. 0 means very dry, 15 very wet. Supports four leaf sensors.
  uint8_t   insideAlarms;      // Currently active inside alarms.
  uint8_t   rainAlarms;        // Currently active rain alarms.
  uint16_t  outsideAlarms;     // Currently active outside alarms.
  uint8_t   extraTempHumAlarms[8]; // Currently active temperature / humidity alarms for up to eight stations.
  uint8_t   soilLeafAlarms[4];     // Currently active soil and leaf alarms for up to four sensors
  uint8_t   transmitterBatteryStatus;  // Transmitter battery status (0 or 1)
  uint16_t  consoleBatteryVoltage; // Console voltage as  ((Data * 300)/512)/100.0
  uint8_t   forecastIcons;         // Forecast icons
  uint8_t   forecastRuleNumber;    // Forecast rule number
  uint16_t  timeOfSunrise;         // Sunrise time stored as hour * 100 + min.
  uint16_t  timeOfSunset;          // Sunset time stored as hour * 100 + min.
  char      lfcr[2];               // Carriage return / line feed
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
  uint16_t  dateStamp;
  uint16_t  timeStamp;
  int16_t   outsideTemp;
  int16_t   highOutTemp;
  int16_t   lowOutTemp;
  uint16_t  rainfall;
  uint16_t  highRainRate;
  uint16_t  barometer;
  uint16_t  solarRad;
  uint16_t  windSamples;
  int16_t   insideTemp;
  uint8_t   insideHum;
  uint8_t   outsideHum;
  uint8_t   avgWindSpd;
  uint8_t   highWindSpd;
  uint8_t   dirHiWindSpd;
  uint8_t   prevWindDir;
  uint8_t   avgUVIndex;
  uint8_t   eT;
  int16_t   highSolarRad;
  uint8_t   highUVIdx;
  uint8_t   forecastRule;
  uint8_t   leafTemp0;
  uint8_t   leafTemp1;
  uint8_t   leafWet0;
  uint8_t   leafWet1;
  uint8_t   soilTemp0;
  uint8_t   soilTemp1;
  uint8_t   soilTemp2;
  uint8_t   soilTemp3;
  uint8_t   recType;
  uint8_t   extraHum0;
  uint8_t   extraHum1;
  uint8_t   extraTemp0;
  uint8_t   extraTemp1;
  uint8_t   extraTemp2;
  uint8_t   soilMoist0;
  uint8_t   soilMoist1;
  uint8_t   soilMoist2;
  uint8_t   soilMoist3;
};

// Fake archive record (temporary)
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

// vim: et:sts=2:ts=2:sw=2

