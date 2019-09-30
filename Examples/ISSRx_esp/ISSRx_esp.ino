// Sample usage of the DavisRFM69 library to sniff the packets from a Davis Instruments
// wireless Integrated Sensor Suite (ISS), demostrating compatibility between the RFM69
// and the TI CC1020 transmitter used in that hardware.  Packets received from the ISS are
// passes through to the serial port.  This code retains some of the debugging
// functionality of the LowPowerLabs Gateway sketch, on which this code is based.
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
// Example released under the MIT License (http://opensource.org/licenses/mit-license.php)
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
//
// This program has been developed on a Moteino R3 Arduino clone with integrated RFM69W
// transceiver module.  Note that RFM12B-based modules will not work.  See the README
// for more details.
// patched to remove spiflash support and add esp8266 support (nodemcu v2)

#include <DavisRFM69.h>
#include <SPI.h>

#include "davismessage.h"
// NOTE: *** One of DAVIS_FREQS_US, DAVIS_FREQS_EU, DAVIS_FREQS_AU, or
// DAVIS_FREQS_NZ MUST be defined at the top of DavisRFM69.h ***

//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED          16  // nodemcu have LEDs on 16
#define SERIAL_BAUD   115200
#define PACKET_INTERVAL 2555
boolean strmon = false;       // Print the packet when received?

DavisRFM69 radio;

void decode_packet(volatile uint8_t *buf)
{
//TODO: properly implement packetdecoders
  byte trans_id=0xff;
  switch(buf[0]& 0xF0){
    case cap_volt:
      int voltage;
      voltage = (((buf[3] * 4) + ((buf[4] && 0xC0) / 64)) / 100);
      Serial.print(F("supercap_voltage="));
      Serial.println(voltage);
      break;
    case uv_index:
      int UVIndex;
      UVIndex = ((((buf[3] << 8) + buf[4]) >> 6) / 50.0);
      Serial.print(F("UVIndex="));
      if (buf[3]!=0xff)
        Serial.println(UVIndex);
      else
        Serial.println(F("no sensor"));
      break;
    case rainrate:
      int rainr;
      if ((buf[4] && 0x40) == 0)
        rainr = (720 / (((buf[4] && 0x30) / 16 * 250) + buf[3]));
      else
        rainr = (11520 / (((buf[4] && 0x30) / 16 * 250) + buf[3]));
      Serial.print(F("rainr="));
      if (buf[3]!=0xff)
        Serial.println(rainr);
      else
        Serial.println(F("no sensor"));
      break;
    case solarrad:
      float Solar_rad;
      Solar_rad = ((((buf[3] << 8) + buf[4]) >> 6) * 1.757936);
      Serial.print(F("Solar_rad="));
      if (buf[3]!=0xff)
        Serial.println(Solar_rad);
      else
        Serial.println(F("no sensor"));
      break;
    case sol_volt:
      int sol_voltage; 
      sol_voltage = (((buf[3] * 4) + ((buf[4] && 0xC0) / 64)) / 100);
      Serial.print(F("solar_voltage="));
      Serial.println(sol_voltage);
      break;
    case temp:
      float tempF;
      tempF = ((buf[3] * 256 + buf[4]) / 160 );
      Serial.print(F("tempF="));
      Serial.print(tempF);
      float tempC;
      tempC = ((tempF-32)/1.8);
      Serial.print(F(" tempC="));
      Serial.println(tempC);
      break;
    case windgust:
      byte gust;
      byte gust_index;
      gust = buf[3];
      gust_index = (buf[5] >> 4);
      Serial.print(F("gust="));
      Serial.print(gust);
      Serial.print(F("gustindex="));
      Serial.println(gust_index);
      break;
    case humi:
      byte humidity;
      humidity = ((((buf[4] >> 4) << 8) + buf[3]) / 10.0);
      Serial.print(F("humidity="));
      Serial.println(humidity);
      break;
    case rain:
      byte rainbuckets;
      rainbuckets = (buf[3]&0x7F);
      Serial.print(F("rainbuckettips="));
      Serial.println(rainbuckets);
      break;
    default:
      Serial.println(F("unknown packet type"));
      break;
  }
  if (buf[0] & batlow)
    Serial.println(F("low battery"));
  byte wind_speed;
  byte wind_dir;
  wind_speed=(buf[1]);
  wind_dir=(buf[2]);
  Serial.print(F("wind speed="));
  Serial.print(wind_speed);
  Serial.print(F(" wind direction="));
  Serial.print(wind_dir);
  trans_id=(buf[0] & tr_id);
  Serial.print(F(" transmitter_id="));
  Serial.println(trans_id);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize();
  radio.setChannel(0);              // Frequency / Channel is *not* set in the initialization. Do it right after.
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  Serial.println(F("\n\nDavisRFM69 Gateway - START"));
  Serial.println(F("\nWaiting for signal in region defined in DavisRFM69.h"));
}

unsigned long lastRxTime = 0;
byte hopCount = 0;

void loop() {
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == 'r') //d=dump all register values
    {
      radio.readAllRegs();
      Serial.println();
    }
    if (input == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
      Serial.print( "Radio Temp is ");
      Serial.print(temperature);
      Serial.print("C, ");
      Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
      Serial.println('F');
    }
  }

  // The check for a zero CRC value indicates a bigger problem that will need
  // fixing, but it needs to stay until the fix is in.
  // TODO Reset the packet statistics at midnight once I get my clock module.
  if (radio.receiveDone()) {
    packetStats.packetsReceived++;
    lastRxTime = millis();
    unsigned int crc = radio.crc16_ccitt(radio.DATA, 6);
    if ((crc == (word(radio.DATA[6], radio.DATA[7]))) && (crc != 0)) {
      packetStats.receivedStreak++;
      hopCount = 1;
      blink(LED,3);
      decode_packet(radio.DATA);
      Serial.print(F("crc ok  "));
    } else {
      Serial.print(F("crcfail "));
      packetStats.crcErrors++;
      packetStats.receivedStreak = 0;
      delay(3);
    }

    if (strmon) printStrm();
#if 1
    Serial.print(radio.CHANNEL);
    Serial.print(F(" - Data: "));
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
      if (radio.DATA[i]<0x10)
        Serial.print(F("0"));
      Serial.print(radio.DATA[i], HEX);
      Serial.print(F(" "));
    }
    Serial.print(F("  RSSI: "));
    Serial.println(radio.RSSI);
    Serial.println();
#endif
    // Whether CRC is right or not, we count that as reception and hop.
    radio.hop();
  }

  // If a packet was not received at the expected time, hop the radio anyway
  // in an attempt to keep up.  Give up after 25 failed attempts.  Keep track
  // of packet stats as we go.  I consider a consecutive string of missed
  // packets to be a single resync.  Thx to Kobuki for this algorithm.
  if ((hopCount > 0) && ((millis() - lastRxTime) > (hopCount * PACKET_INTERVAL + 200))) {
    packetStats.packetsMissed++;
    if (hopCount == 1) packetStats.numResyncs++;
    if (++hopCount > 25) hopCount = 0;
    radio.hop();
    Serial.print(F("resync to ch "));
    Serial.println(radio.CHANNEL);
  }
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

void blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);
  digitalWrite(PIN,HIGH);
}
