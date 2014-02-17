// DHT11 & 22 sensor interface extracted from...
// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include "DHTxx.h"
// #include <avr/sleep.h>
#include <util/atomic.h>

#define DEBUG_DHT 0 // add code to send info over the serial port of non-zero

DHTxx::DHTxx (byte pinNum) : pin (pinNum) {
  digitalWrite(pin, HIGH);
}

bool DHTxx::reading (int& temp, int &humi, bool precise) {
  pinMode(pin, OUTPUT);
  delay(10); // wait for any previous transmission to end
  digitalWrite(pin, LOW);
  delay(18);
  
  cli();
  
  digitalWrite(pin, HIGH);
  delayMicroseconds(30);
  pinMode(pin, INPUT);
  
  byte data[6]; // holds a few start bits and then the 5 real payload bytes
#if DEBUG_DHT
  static byte times[48];
  memset(times, 0, sizeof times);
#endif

  // each bit is a high edge followed by a var-length low edge
  for (byte i = 7; i < 48; ++i) {
    // wait for the high edge, then measure time until the low edge
    byte timer;
    for (byte j = 0; j < 2; ++j)
      for (timer = 0; timer < 250; ++timer)
        if (digitalRead(pin) != j)
          break;
#if DEBUG_DHT
    times[i] = timer;
#endif
    // if no transition was seen, return 
    if (timer >= 250) {
      sei();
      return false;
    }
    // collect each bit in the data buffer
    byte offset = i / 8;
    data[offset] <<= 1;
    data[offset] |= timer > 7;
  }
  
  sei();

#if DEBUG_DHT
  Serial.print("DHT");
  for (byte i = 7; i < 48; ++i) {
    Serial.print(' ');
    Serial.print(times[i]);
  }
  Serial.println();
  Serial.print("HEX");
  for (byte i = 0; i < sizeof data; ++i) {
    Serial.print(' ');
    Serial.print(data[i], HEX);
  }
  Serial.print(" : ");
  byte s = data[1] + data[2] + data[3] + data[4];
  Serial.print(s, HEX);
  Serial.println();
#endif
  
  byte sum = data[1] + data[2] + data[3] + data[4];
  if (sum != data[5])
    return false;
  
  humi = precise ? (data[1] << 8) | data[2] : 10 * data[1];

  word t = precise ? ((data[3] & 0x7F) << 8) | data[4] : 10 * data[3];
  temp = data[3] & 0x80 ? - t : t;

  return true;
}
