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

#include <RFM69.h>
#include <DavisRFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

// Match frequency to the hardware version of the radio on your Moteino (uncomment one):
// Note: Only the 915 MHz North American version is supported at this moment.
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED           9  // Moteinos have LEDs on D9
#define SERIAL_BAUD   115200

DavisRFM69 radio;
SPIFlash flash(8, 0xEF30); //EF40 for 16mbit windbond chip

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,0,0);  // 0, 0 is nodeID and networkID, always zero in this implementation.
  radio.setChannel(0);              // Channel is *not* set in the initialization. Do it right after.
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL! (is chip present?)");
}

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
    if (input == 'd') //d=dump flash area
    {
      Serial.println("Flash content:");
      int counter = 0;

      while(counter<=256){
        Serial.print(flash.readByte(counter++), HEX);
        Serial.print('.');
      }
      while(flash.busy());
      Serial.println();
    }
    if (input == 'e')
    {
      Serial.print("Erasing Flash chip ... ");
      flash.chipErase();
      while(flash.busy());
      Serial.println("DONE");
    }
    if (input == 'i')
    {
      Serial.print("DeviceID: ");
      word jedecid = flash.readDeviceId();
      Serial.println(jedecid, HEX);
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

  if (radio.receiveDone())
  {
    Serial.print(radio.hopIndex); Serial.print(" - ");
    Serial.print("Data: ");
    for (byte i = 0; i < radio.DATALEN; i++) {
      radio.DATA[i] = radio.DATA[i];
      Serial.print(radio.DATA[i], HEX);
      Serial.print(" ");
    }
    Serial.print("  RSSI: ");Serial.print(radio.RSSI);Serial.print("dBm   CRC: ");
    unsigned int crc = radio.crc16_ccitt(radio.DATA, 6);
    Serial.print(crc, HEX);
    Serial.println();
    radio.hop();

    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
