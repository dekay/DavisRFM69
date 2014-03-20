// Sample usage of the DavisRFM69 library to sniff the packets from a Davis Instruments
// wireless Integrated Sensor Suite (ISS), demostrating compatibility between the RFM69
// and the TI CC1020 transmitter used in that hardware.  Packets received from the ISS are
// translated into a format compatible with the Davis Vantage Pro2 (VP2) and Vantage Vue
// consoles.  This example also reads BMP085 and DHT22 sensors connected directly to the
// Moteino (see below).
// See http://madscientistlabs.blogspot.com/2014/02/build-your-own-davis-weather-station_17.html

// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
// Example released under the MIT License (http://opensource.org/licenses/mit-license.php)
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/

// This program has been developed on a Moteino R3 Arduino clone with integrated RFM69W
// transceiver module.  Note that RFM12B-based modules will not work.  See the README
// for more details.

#include <DavisRFM69.h>
#include <DHTxx.h>
#include <SPI.h>
#include <SPIFlash.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SerialCommand.h>
#include <RTC_DS3231.h>       // From https://github.com/mizraith/RTClib

// NOTE: *** One of DAVIS_FREQS_US, DAVIS_FREQS_EU, DAVIS_FREQS_AU, or
// DAVIS_FREQS_NZ MUST be defined at the top of DavisRFM69.h ***

//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED           9  // Moteinos have LEDs on D9
#define SERIAL_BAUD   19200        // Davis console is 19200 by default
#define SENSOR_POLL_INTERVAL 5000  // Read indoor sensors every minute. Console uses 60 seconds

#define DHT_DATA_PIN  4  // Use pin D4 to talk to the DHT22
#define LATITUDE -1071   // Station latitude in tenths of degrees East
#define LONGITUDE 541    // Station longitude in tenths of degrees North
#define ELEVATION 800    // Station height above sea level in feet

#define PACKET_INTERVAL 2555
#define LOOP_INTERVAL 2500

boolean strmon = false;       // Print the packet when received?
DavisRFM69 radio;
SPIFlash flash(8, 0xEF30); //EF40 for 16mbit windbond chip
Adafruit_BMP085 bmp;
DHTxx tempHum(DHT_DATA_PIN);
RTC_DS3231 RTC;
SerialCommand sCmd;

byte loopData[LOOP_PACKET_LENGTH] = {
  'L', 'O', 'O', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Loop packet bytes  0 - 15
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        // Loop packet bytes 16 - 31
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        // Loop packet bytes 32 - 48
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        // Loop packet bytes 48 - 63
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        // Loop packet bytes 64 - 79
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\n',     // Loop packet bytes 80 - 95
  '\r', 0, 0                                             // Loop packet bytes 96 - 98
};

unsigned int packetStats[PACKET_STATS_LENGTH] = {0, 0, 0, 0 ,0};

volatile bool oneMinutePassed = false;

void rtcInterrupt(void) {
  oneMinutePassed = true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize();
  radio.setChannel(0);              // Frequency / Channel is *not* set in the initialization. Do it right after.
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif

  // Set up BMP085 pressure and temperature sensor
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
    while (1) { }
  }

  // Set up DS3231 real time clock on Moteino INT1
  Wire.begin();
  RTC.begin();

  if (! RTC.isrunning()) {
    Serial.println(F("Could not find a valid DS3231 RTC, check wiring and ensure battery is installed!"));
    while (1) { }
  }

  RTC.enable32kHz(false);   // We don't even connect this pin
  RTC.SQWEnable(false);     // Disabling the square wave enables the alarm interrupt
  setAlarming();
  attachInterrupt(1, rtcInterrupt, FALLING);

  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("BARDATA", cmdBardata);   // Barometer calibration data
  sCmd.addCommand("DUMPREG", cmdDumpreg);   // Dump radio registers - undocumented command
  sCmd.addCommand("EEBRD", cmdEebrd);       // EEPROM Read
  sCmd.addCommand("HILOWS", cmdHiLows);     // Send the HILOW data
  sCmd.addCommand("LOOP", cmdLoop);         // Send the loop data
  sCmd.addCommand("NVER", cmdNver);         // Send the version string
  sCmd.addCommand("RXCHECK", cmdRxcheck);   // Receiver Stats check
  sCmd.addCommand("STRMOFF", cmdStrmoff);   // Disable printing of received packet data
  sCmd.addCommand("STRMON", cmdStrmon);     // Enable printing of received packet data
  sCmd.addCommand("TEST", cmdTest);         // Echo's "TEST"
  sCmd.addCommand("VER", cmdVer);           // Send the associated date for this version
  sCmd.addCommand("WRD\x12M", cmdWRD);      // Support the Davis legacy "WRD" command
  sCmd.addCommand("GETTIME", cmdGettime);   // Send back current RTC date/time
  sCmd.addCommand("SETTIME", cmdSettime);   // Update current RTC date/time from PC
  sCmd.addCommand("DMPAFT", cmdDmpaft);     // Download archive records after date:time specified
  sCmd.setDefaultHandler(cmdUnrecognized);  // Handler for command that isn't matched
  sCmd.setNullHandler(cmdWake);             // Handler for an empty line to wake the simulated console
}

// See https://github.com/dekay/im-me/blob/master/pocketwx/src/protocol.txt
// Console update rates: * indicates what is done, X indicates ignored, ? indicates in progress
// - Clock display updates once per minute
// * Barometric pressure once per minute
// - Dewpoint every ten to twelve seconds (calculated)
// - Forecast, evapotranspiration, and heat index every hour
// * Inside humidity every minute
// * Outside humidity every 50 to 60 seconds
// - Leaf wetness every 46 to 64 seconds
// - Rainfall and rain rate every 20 to 24 seconds
// - Soil moisture every 77 to 90 seconds
// - UV, UV Index and solar radiation every 50 to 60 secondes (5 minutes when dark)
// * Inside temperature every minute
// * Outside temperature every 10 to 12 seconds
// X Extra temperature sensors or probes every 10 to 12 seconds
// - Temperature humidity sun wind index every 10 to 12 seconds
// - Wind chill every 10 to 12 seconds (calculated)
// ? Wind direction every 2.5 - 3 seconds
// * Wind speed every 2.5 - 3 seconds
// - Wind speed 10 minute average every minute

long lastPeriod = -1;
unsigned long lastRxTime = 0;
unsigned long lastLoopTime = 0;
byte hopCount = 0;
unsigned int loopCount = 0;

void loop() {
  if (Serial.available() > 0) loopCount = 0; // if we receive anything while sending LOOP packets, stop the stream
  sendLoopPacket(); // send out a LOOP packet if needed

  if (oneMinutePassed) clearAlarmInterrupt();

  sCmd.readSerial(); // Process serial commands

  int currPeriod = millis()/SENSOR_POLL_INTERVAL;

  if (currPeriod != lastPeriod) {
    lastPeriod=currPeriod;
    readInsideTempHum();
    readPressure();
  }

  // The check for a zero CRC value indicates a bigger problem that will need
  // fixing, but it needs to stay until the fix is in.
  // TODO Reset the packet statistics at midnight once I get my clock module.
  if (radio.receiveDone()) {
    packetStats[PACKETS_RECEIVED]++;
    unsigned int crc = radio.crc16_ccitt(radio.DATA, 6);
    if ((crc == (word(radio.DATA[6], radio.DATA[7]))) && (crc != 0)) {
      processPacket();
      packetStats[RECEIVED_STREAK]++;
      hopCount = 1;
      blink(LED,3);
    } else {
      packetStats[CRC_ERRORS]++;
      packetStats[RECEIVED_STREAK] = 0;
    }

    if (strmon) printStrm();
#if 0
    Serial.print(radio.CHANNEL);
    Serial.print(F(" - Data: "));
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
      Serial.print(radio.DATA[i], HEX);
      Serial.print(F(" "));
    }
    Serial.print(F("  RSSI: "));
    Serial.println(radio.RSSI);
#endif
    // Whether CRC is right or not, we count that as reception and hop.
    lastRxTime = millis();
    radio.hop();
  }

  // If a packet was not received at the expected time, hop the radio anyway
  // in an attempt to keep up.  Give up after 25 failed attempts.  Keep track
  // of packet stats as we go.  I consider a consecutive string of missed
  // packets to be a single resync.  Thx to Kobuki for this algorithm.
  if ((hopCount > 0) && ((millis() - lastRxTime) > (hopCount * PACKET_INTERVAL + 200))) {
    packetStats[PACKETS_MISSED]++;
    if (hopCount == 1) packetStats[NUM_RESYNCS]++;
    if (++hopCount > 25) hopCount = 0;
    radio.hop();
  }
}

// Read the data from the ISS and figure out what to do with it
void processPacket() {
  // Every packet has wind speed, direction, and battery status in it
  loopData[WIND_SPEED] = radio.DATA[1];
#if 0
  Serial.print("Wind Speed: ");
  Serial.print(loopData[WIND_SPEED);
  Serial.print("  Rx Byte 1: ");
  Serial.println(radio.DATA[1]);
#endif

  // There is a dead zone on the wind vane. No values are reported between 8
  // and 352 degrees inclusive. These values correspond to received byte
  // values of 1 and 255 respectively
  // See http://www.wxforum.net/index.php?topic=21967.50
  unsigned int windDirection = 9 + radio.DATA[2] * 342.0f / 255.0f;
  loopData[WIND_DIRECTION_LSB] = lowByte(windDirection);
  loopData[WIND_DIRECTION_MSB] = highByte(windDirection);

#if 0
  Serial.print(F("Wind Direction: "));
  Serial.print(windDirection);
  Serial.print(F("  MSB: "));
  Serial.print(loopData[WIND_DIRECTION_MSB]);
  Serial.print(F("  LSB: "));
  Serial.print(loopData[WIND_DIRECTION_LSB]);
  Serial.print(F("  Rx Byte 2: "));
  Serial.println(radio.DATA[2]);
#endif

  loopData[TRANSMITTER_BATTERY_STATUS] = (radio.DATA[0] & 0x8) >> 3;
#if 0
  Serial.print("Battery status: ");
  Serial.println(loopData[TRANSMITTER_BATTERY_STATUS]);
#endif

  // Now look at each individual packet. Mask off the four low order bits. The highest order bit of the
  // four is set high when the ISS battery is low.  The low order three bits are the station ID.
  int outsideTemp;

  switch (radio.DATA[0] & 0xf0) {
  case 0x80:
    outsideTemp = (int)(word(radio.DATA[3], radio.DATA[4])) >> 4;
    loopData[OUTSIDE_TEMPERATURE_LSB] = lowByte(outsideTemp);
    loopData[OUTSIDE_TEMPERATURE_MSB] = highByte(outsideTemp);
#if 0
    Serial.print(F("Outside Temp: "));
    Serial.print(outsideTemp);
    Serial.print(F("  MSB: "));
    Serial.print(loopData[OUTSIDE_TEMPERATURE_MSB]);
    Serial.print(F("  LSB: "));
    Serial.print(loopData[OUTSIDE_TEMPERATURE_LSB]);
    Serial.print(F("  Rx Byte 3: "));
    Serial.print(radio.DATA[3]);
    Serial.print(F("  Rx Byte 4: "));
    Serial.println(radio.DATA[4]);
#endif
    break;
  case 0xa0:
    loopData[OUTSIDE_HUMIDITY] = (float)(word((radio.DATA[4] >> 4), radio.DATA[3])) / 10.0;
#if O
    Serial.print("Outside Humdity: ");
    Serial.print(loopData[OUTSIDE_HUMIDITY]);
    Serial.print("  Rx Byte 3: ");
    Serial.print(radio.DATA[3]);
    Serial.print("  Rx Byte 4: ");
    Serial.println(radio.DATA[4]);
#endif
    break;
    // default:
  }
#if 0
  printFreeRam();
#endif
}

// Indoor temperature and humidity is read once per minute
void readInsideTempHum() {
  int insideTempF, insideTempC, insideHumidity;
  if (tempHum.reading(insideTempC, insideHumidity, true)) {
    // Temperature and humidity are returned in tenths of a deg C and tenths of a percent, respectively
    // Values out of the console are tenths of a deg F and integer percent values.  PITA.
    insideTempF = insideTempC*1.8 + 320;
    loopData[INSIDE_TEMPERATURE_MSB] =highByte(insideTempF);
    loopData[INSIDE_TEMPERATURE_LSB] =lowByte(insideTempF);
    loopData[INSIDE_HUMIDITY] = (insideHumidity + 5) * 0.1;  // Round the reading
#if 0
    Serial.print(F("Inside TempF: "));
    Serial.print(insideTempF);
    Serial.print(F("  MSB: "));
    Serial.print(loopData[INSIDE_TEMPERATURE_MSB]);
    Serial.print(F("  LSB: "));
    Serial.println(loopData[INSIDE_TEMPERATURE_LSB]);
    printFreeRam();
#endif
#if 0
    Serial.print(F("Inside Humidity Raw: "));
    Serial.print(insideHumidity);
    Serial.print(F("  Loop Value: "));
    Serial.println(loopData[INSIDE_HUMIDITY]);
    printFreeRam();
#endif
  }
}

// Barometric pressure is read once per minute
// Units from the BMP085 are Pa.  Need to convert to thousandths of inches of Hg.
void readPressure() {
  unsigned int pressureHg;
  pressureHg = bmp.readPressure()*0.29529987508;
  loopData[BAROMETER_MSB] =highByte(pressureHg);
  loopData[BAROMETER_LSB] =lowByte(pressureHg);
#if 0
  Serial.print(F("Pressure: "));
  Serial.print(pressureHg);
  Serial.print(F("  MSB: "));
  Serial.print(loopData[BAROMETER_MSB]);
  Serial.print(F("  LSB: "));
  Serial.println(loopData[BAROMETER_LSB]);
  printFreeRam();
#endif
}

//--- Console command emulation ---//

void cmdBardata() {
  // TODO Fix dew point, virtual temp, C, and R
  // TODO Allow entry of Elevation and BARCAL
  printOk();
  Serial.print(F("BAR "));
  Serial.print(word(loopData[BAROMETER_MSB], loopData[BAROMETER_LSB]));
  Serial.print(F("\n\rELEVATION "));
  Serial.print(ELEVATION);
  Serial.print(F("\n\rDEW POINT "));
  Serial.print(word(loopData[OUTSIDE_TEMPERATURE_MSB], loopData[OUTSIDE_TEMPERATURE_LSB]));
  Serial.print(F("\n\rVIRTUAL TEMP "));
  Serial.print(word(loopData[OUTSIDE_TEMPERATURE_MSB], loopData[OUTSIDE_TEMPERATURE_LSB]));
  Serial.print(F("\n\rC "));
  Serial.print(29);
  Serial.print(F("\n\rR "));
  Serial.print(1001);
  Serial.print(F("\n\rBARCAL "));
  Serial.print(0);
  // These factory cal values have no meaning in our implementation
  Serial.print(F("\n\rGAIN 1\n\rOFFSET 0\n\r"));
}

void cmdDumpreg() {
  radio.readAllRegs();
  Serial.println();
}

void cmdEebrd() {
  char *cmdMemLocation, *cmdNumBytes;
  byte response[2] = {0, 0};
  byte responseLength = 0;
  bool validCommand = true;

  cmdMemLocation = sCmd.next();
  cmdNumBytes = sCmd.next();
  if ((cmdMemLocation != NULL) && (cmdNumBytes != NULL)) {
    switch (strtol(cmdMemLocation, NULL, 16)) {
    case EEPROM_LATITUDE_LSB:
      response[0] = lowByte(LATITUDE);
      response[1] = highByte(LATITUDE);
      responseLength = 2;
      break;
    case EEPROM_LONGITUDE_LSB:
      response[0] = lowByte(LONGITUDE);
      response[1] = highByte(LONGITUDE);
      responseLength = 2;
      break;
    case EEPROM_ELEVATION_LSB:
      response[0] = lowByte(ELEVATION);
      response[1] = highByte(ELEVATION);
      responseLength = 2;
      break;
    case EEPROM_TIME_ZONE_INDEX:
      response[0] = GMT_ZONE_MINUS600;
      responseLength = 1;
      break;
    case EEPROM_DST_MANAUTO:
      response[0] = DST_USE_MODE_MANUAL;
      responseLength = 1;
      break;
    case EEPROM_DST_OFFON:
      response[0] = DST_SET_MODE_STANDARD;
      responseLength = 1;
      break;
    case EEPROM_GMT_OFFSET_LSB:
      // TODO GMT_OFFSETS haven't been calculated yet. Zero for now.
      response[0] = lowByte(GMT_OFFSET_MINUS600);
      response[1] = highByte(GMT_OFFSET_MINUS600);
      responseLength = 2;
      break;
    case EEPROM_GMT_OR_ZONE:
      response[0] = GMT_OR_ZONE_USE_INDEX;
      responseLength = 1;
      break;
    case EEPROM_UNIT_BITS:  // Memory location for setup bits. Assume one byte is being asked for.
      response[0] = BAROMETER_UNITS_IN | TEMP_UNITS_TENTHS_F | ELEVATION_UNITS_FEET | RAIN_UNITS_IN | WIND_UNITS_MPH;
      responseLength = 1;
      break;
    case EEPROM_SETUP_BITS:  // Memory location for setup bits. Assume one byte is being asked for.
      // TODO The AM / PM indication isn't set yet.  Need my clock chip first.
      response[0] = LONGITUDE_WEST | LATITUDE_NORTH | RAIN_COLLECTOR_01IN | WIND_CUP_LARGE | MONTH_DAY_MONTHDAY | AMPM_TIME_MODE_24H;
      responseLength = 1;
      break;
    case EEPROM_RAIN_YEAR_START:
      response[0] = RAIN_SEASON_START_JAN;
      responseLength = 1;
      break;
    case EEPROM_ARCHIVE_PERIOD:
      // TODO We don't actually implement archiving yet, and when we do, this value needs to sync
      // with the actual archive period being used.
      response[0] = ARCHIVE_PERIOD_MINS_10;
      responseLength = 1;
      break;
    default:
      validCommand = false;
      printNack();
    }

    if (validCommand) {
      unsigned int crc = radio.crc16_ccitt(response, responseLength);
      printAck();
      for (byte i = 0; i < responseLength; i++)
        Serial.write(response[i]);
      Serial.write(highByte(crc));
      Serial.write(lowByte(crc));
    }
  } else {
    printNack();
  }
}

// We just don't have enough memory to support this.  Fail.
void cmdHiLows() {
  byte count = 0;
  printAck();
  for (int i = 0; i < HI_LOWS_LENGTH + 2; i++) Serial.write(0);
}

void cmdLoop() {
  char *loops;
  lastLoopTime = 0;
  printAck();
  loops = sCmd.next();
  if (loops != NULL) {
    loopCount = strtol(loops, NULL, 10);
  } else {
    loopCount = 1;
  }
}

void sendLoopPacket() {
  if (loopCount <= 0 || millis() - lastLoopTime < LOOP_INTERVAL) return;
  lastLoopTime = millis();
  loopCount--;
  // Calculate the CRC over the entire length of the current packet with the exception of the
  // last two bytes in the packet that hold the CRC itself.
  unsigned int crc = radio.crc16_ccitt(loopData, LOOP_PACKET_LENGTH - 2);
  loopData[CRC_MSB]=highByte(crc);
  loopData[CRC_LSB]=lowByte(crc);
  for (byte i = 0; i < LOOP_PACKET_LENGTH; i++) Serial.write(loopData[i]);
}

void cmdNver() {
  printOk();
  Serial.print(F("1.90\n\r"));
}

// Print receiver stats
void cmdRxcheck() {
  printAck();
  printOk();
  for (byte i = 0; i < PACKET_STATS_LENGTH; i++) {
    Serial.print(F(" "));          // Note. The real console has this leading space.
    Serial.print(packetStats[i]);
  }
  Serial.print(F("\n\r"));
}

void cmdStrmoff() {
  printOk();
  strmon =  false;
}

void cmdStrmon() {
  printOk();
  strmon =  true;
}

void cmdTest() {
  Serial.print(F("TEST\n"));
}

void cmdVer() {
  printOk();
  Serial.print(F("Sep 29 2009\n\r"));
}

void cmdWRD() {
  printAck();
  Serial.write(16);
}

// Gettime and Settime use a binary format as follows:
// seconds - minutes - hours24 - day - month - (year-1900)
// Example (to set 3:27:00 pm, June 4, 2003):
// >"SETTIME"<LF>
// <<ACK>
// ><0><27><15><4><6><103><2 Bytes<<ACK><2 bytes of CRC>
// <<ACK>
void cmdGettime() {
  byte davisDateTime[6];
  printAck();
  DateTime now = RTC.now();
  davisDateTime[0] = now.second();
  davisDateTime[1] = now.minute();
  davisDateTime[2] = now.hour();
  davisDateTime[3] = now.day();
  davisDateTime[4] = now.month();
  davisDateTime[5] = now.year() - 1900;
#if 0
  Serial.print(F("The time in reverse Davis format is now"));
  for (int8_t i = 5; i >= 0; i--) {
    Serial.print(F(" "));
    Serial.print(davisDateTime[i]);
  }
  Serial.println();
#endif
  unsigned int crc = radio.crc16_ccitt(davisDateTime, 6);
  Serial.write(davisDateTime, 6);
  Serial.write(highByte(crc));
  Serial.write(lowByte(crc));
}

void cmdSettime() {
  byte davisDateTime[8];
  printAck();
  // delay(2000);    Why were these delays here if read() is blocking? Kobuki code bug?
  // Read six bytes for time and another two bytes for CRC
  for (byte i = 0; i < 8; i++) {
    davisDateTime[i] = Serial.read();
    // delay(200);
  }

  // Set the time only if the CRC is OK
  unsigned int crc = radio.crc16_ccitt(davisDateTime, 6);
  if (crc == (word(davisDateTime[6], davisDateTime[7]))) {
    printAck();
    RTC.adjust(DateTime(davisDateTime[5] + 1900, davisDateTime[4], davisDateTime[3], \
      davisDateTime[2], davisDateTime[1], davisDateTime[0]));
  } else {
    printNack();
  }
}

void cmdDmpaft() {
  printAck();
  // read 2 byte vantageDateStamp, the 2 byte vantageTimeStamp, and a 2 byte CRC
  while (Serial.available() <= 0);
  for (byte i = 0; i < 6; i++) Serial.read();
  printAck();

  // From Davis' docs:
  //   Each archive record is 52 bytes. Records are sent to the PC in 264 byte pages. Each page
  //   contains 5 archive records and 4 unused bytes.

  // send the number of "pages" that will be sent (2 bytes), the location within the first page of the first record, and 2 Byte CRC
  byte response[4] = { 1, 0, 0, 0 }; // L,H;L,H -- 1 page; first record is #0
  unsigned int crc = radio.crc16_ccitt(response, 4);
  for (byte i = 0; i < 4; i++)
    Serial.write(response[i]);
  Serial.write(highByte(crc));
  Serial.write(lowByte(crc));
  while (Serial.available() <= 0);
  if (Serial.read() != 0x06); // should return if condition is true, but can't get a 0x06 here for the life of me, even if weewx does send it...

  // The format of each page is:
  // 1 Byte sequence number (starts at 0 and wraps from 255 back to 0)
  // 52 Byte Data record [5 times]
  // 4 Byte unused bytes
  // 2 Byte CRC
  response[0] = 0;
  crc = radio.crc16_ccitt(response, 1);
  Serial.write(0);
  byte * farp = (byte *)&fakeArchiveRec;
  for (byte i = 0; i < 5; i++) {
    crc = radio.crc16_ccitt(farp, sizeof(fakeArchiveRec), crc);
    for (byte j = 0; j < sizeof(fakeArchiveRec); j++) Serial.write(farp[j]);
  }
  for (byte i = 0; i < 4; i++) Serial.write(0);
  crc = radio.crc16_ccitt(response, 4, crc);
  Serial.write(highByte(crc));
  Serial.write(lowByte(crc));
}

void cmdWake() {
  Serial.print(F("\n\r"));
}

// This gets set as the default handler, and gets called when no other command matches.
void cmdUnrecognized(const char *command) {
  printOk();
}

//--- Print related support functions ---//
void printAck() {
  Serial.write(0x06);
}

void printNack() {
  Serial.write(0x21);
}

// From http://jeelabs.org/2011/05/22/atmega-memory-use/
void printFreeRam() {
  extern int __heap_start, *__brkval;
  int v;
  Serial.print(F("Free mem: "));
  Serial.println((int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
}

void printOk() {
  Serial.print(F("\n\rOK\n\r"));
}

void printStrm() {
  for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(radio.DATA[i], HEX);
    Serial.print(F("\n\r"));
  }
  Serial.print(F("\n\r"));
}

//--- Ancillary functions ---//
void blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

// Super ugly. Wanted to get this working but running out of time.  Make pretty later.
// This enables an interrupt every minute.  We will use this for reading sensors on the
// minute and enabling the archiving on multiples of a minute.
void setAlarming()
{
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(static_cast<uint8_t>(DS3231_REG_CONTROL));
  Wire.endTransmission();

  // control register
  Wire.requestFrom(DS3231_ADDRESS, 1);
  uint8_t creg = Wire.read();     //do we need the bcd2bin

  creg &= ~0b00000001; // Clear INTCN and A1IE to enable alarm interrupt but disable Alarm1
  creg |=  0b00000110; // Set A2IE to enable Alarm2

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(static_cast<uint8_t>(DS3231_REG_CONTROL));
  Wire.write(static_cast<uint8_t>(creg));
  Wire.endTransmission();

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(static_cast<uint8_t>(DS3231_REG_A2DAYDATE));
  Wire.write(static_cast<uint8_t>(0b10000000));
  Wire.endTransmission();

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(static_cast<uint8_t>(DS3231_REG_A2HOURS));
  Wire.write(static_cast<uint8_t>(0b10000000));
  Wire.endTransmission();

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(static_cast<uint8_t>(DS3231_REG_A2MINUTES));
  Wire.write(static_cast<uint8_t>(0b10000000));
  Wire.endTransmission();
}

void clearAlarmInterrupt()
{
  oneMinutePassed = false;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(static_cast<uint8_t>(DS3231_REG_STATUS_CTL));
  Wire.write(static_cast<uint8_t>(0b00000000));
  Wire.endTransmission();
}
