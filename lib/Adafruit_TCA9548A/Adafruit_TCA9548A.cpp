/*!
 * @file Adafruit_TCA9548A.cpp
 *
 * @mainpage Adafruit TCA9548A I2C Multiplexer
 *
 * @section intro_sec Introduction
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
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_TCA9548A.h"

/*!
 * @brief  Instantiates a new TCA9548A class
 * @param  addr
 * the I2C address of the multiplexer
 */
Adafruit_TCA9548A::Adafruit_TCA9548A(uint8_t addr) { _addr = addr; }

/*!
 * @brief  Begins communication with the TCA9548A
 * @param  wire
 * the TwoWire object to use
 * @return true if device is found, false otherwise
 */
bool Adafruit_TCA9548A::begin(TwoWire *wire) {
  _wire = wire;
  _wire->begin();
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission() > 0)
    return false;
  return true;
}

/*!
    @brief  checks if a TCA9548A is connected
    @return true if a TCA9548A is connected, false otherwise
*/
bool Adafruit_TCA9548A::isConnected() {
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission() > 0)
    return false;
  return true;
}

/*!
 * @brief  Selects the I2C channel to communicate on
 * @param  i
 * the channel to select, from 0 to 7
 */
void Adafruit_TCA9548A::selectChannel(uint8_t i) {
  if (i > 7)
    return;
  _wire->beginTransmission(_addr);
  _wire->write(1 << i);
  _wire->endTransmission();
}

/*!
 * @brief  Deselects all I2C channels
 */
void Adafruit_TCA9548A::deselectAll() {
  _wire->beginTransmission(_addr);
  _wire->write(0);
  _wire->endTransmission();
}