/*!
 * @file Adafruit_TCA9548A.h
 *
 * This is a library for the Adafruit TCA9548A I2C Multiplexer
 *
 * Designed specifically to work with the Adafruit TCA9548A.
 *
 * Pick one up today in the adafruit shop!
 * ------> https://www.adafruit.com/product/2717
 *
 * These displays use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit andopen-source hardware by purchasing products
 * from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * BSD license, all text above must be included in any redistribution
 */

#ifndef _ADAFRUIT_TCA9548A_H
#define _ADAFRUIT_TCA9548A_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define TCA9548A_ADDR (0x70) ///< Default I2C address

/*!
 * @brief  Class that stores state and functions for interacting with
 * TCA9548A I2C Multiplexer
 */
class Adafruit_TCA9548A {
public:
  Adafruit_TCA9548A(uint8_t addr = TCA9548A_ADDR);
  bool begin(TwoWire *wire = &Wire);
  bool isConnected();
  void selectChannel(uint8_t i);
  void deselectAll();

private:
  uint8_t _addr;
  TwoWire *_wire;
};

#endif