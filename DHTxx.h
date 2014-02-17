// DHT11 & 22 sensor interface extracted from...
// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#ifndef DHTXX_H
#define DHTXX_H
#include <Arduino.h> // Arduino 1.0
#include <stdint.h>
#include <avr/pgmspace.h>

/// Interface for the DHT11 and DHT22 sensors, does not use floating point
class DHTxx {
  byte pin;
public:
  DHTxx (byte pinNum);
  /// Results are returned in tenths of a degree and percent, respectively.
  /// Set "precise" to true for the more accurate DHT21 and DHT22 sensors.
  bool reading (int& temp, int &humi, bool precise =false);
};

#endif  // DHTXX_H
