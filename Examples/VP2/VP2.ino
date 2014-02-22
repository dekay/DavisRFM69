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

#include <RFM69.h>
#include <DavisRFM69.h>
#include <DHTxx.h>
#include <SPI.h>
#include <SPIFlash.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SerialCommand.h>

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

boolean strmon = false;      // Print the packet when received?
DavisRFM69 radio;
SPIFlash flash(8, 0xEF30); //EF40 for 16mbit windbond chip
Adafruit_BMP085 bmp;
DHTxx tempHum(DHT_DATA_PIN);
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(0,0,0);  // 0, 0, 0 is Frequency, nodeID and networkID, always zero in this implementation.
  radio.setChannel(0);              // Frequency / Channel is *not* set in the initialization. Do it right after.
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
#if 0
  if (flash.initialize())
    Serial.println(F("SPI Flash Init OK!"));
  else
    Serial.println(F("SPI Flash Init FAIL! (is chip present?)"));
#endif
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
    while (1) { }
  }

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
long lastRxTime = 0;
byte hopCount = 0;
unsigned int packetInterval = 2555;

void loop() {
  sCmd.readSerial();     // Process serial commands

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
      lastRxTime = millis();
      radio.hop();
      blink(LED,3);
    } else {
      packetStats[CRC_ERRORS]++;
      packetStats[RECEIVED_STREAK] = 0;
    }

    if (strmon) printStrm();
#if 0
    Serial.print(radio.hopIndex);
    Serial.print(F(" - Data: "));
    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print(radio.DATA[i], HEX);
      Serial.print(F(" "));
    }
    Serial.print(F("  RSSI: "));
    Serial.println(radio.RSSI);
#endif
  }

  // If a packet was not received at the expected time, hop the radio anyway
  // in an attempt to keep up.  Give up after 25 failed attempts.  Keep track
  // of packet stats as we go.  I consider a consecutive string of missed
  // packets to be a single resync.  Thx to Kobuki for this algorithm.
  if ((hopCount > 0) && ((millis() - lastRxTime) > (hopCount * packetInterval + 200))) {
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

  // TODO This should be right but it isn't.  It doesn't quite agree with the console.
  unsigned int windDirection = radio.DATA[2] * 360.0 / 255.0 - 7;
  loopData[WIND_DIRECTION_LSB] = lowByte(windDirection);
  loopData[WIND_DIRECTION_MSB] = highByte(windDirection);

#if 0
  Serial.print("Wind Direction: ");
  Serial.print(windDirection);
  Serial.print("  MSB: ");
  Serial.print(loopData[WIND_DIRECTION_MSB]);
  Serial.print("  LSB: ");
  Serial.print(loopData[WIND_DIRECTION_LSB]);
  Serial.print("  Rx Byte 2: ");
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
    case 0x0B:  // Memory location for latitude. Assume two bytes are being asked for.
      response[0] = lowByte(LATITUDE);
      response[1] = highByte(LATITUDE);
      responseLength = 2;
      break;
    case 0x0D:  // Memory location for longitude. Assume two bytes are being asked for.
      response[0] = lowByte(LONGITUDE);
      response[1] = highByte(LONGITUDE);
      responseLength = 2;
      break;
    case 0x2B:  // Memory location for setup bits. Assume one byte is being asked for.
      // TODO DON'T HARDCODE SETUP BITS
      response[0] = 0b01000001;
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
  printAck();
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
  for (byte i = 0; i < radio.DATALEN; i++) {
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
