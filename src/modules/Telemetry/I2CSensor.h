#pragma once

#include <Arduino.h>
#include <Wire.h>

class I2CSensor {
protected:
    uint8_t address;
    TwoWire &wire;
    bool isDetected = false;
    uint8_t variant = 0; // for multiplexer channel or sensor variant

    bool readBytes(uint8_t reg, uint8_t *buffer, size_t len);
    bool writeBytes(uint8_t reg, uint8_t *buffer, size_t len);

public:
    I2CSensor(uint8_t addr, TwoWire &w, uint8_t var = 0) : address(addr), wire(w), variant(var) {}

    virtual void init() = 0; // Pure virtual function, must be implemented by derived classes
    virtual bool hasSensor() { return isDetected; }
    virtual bool isPresent(); // Checks if the sensor responds on the I2C bus
    virtual int32_t getPollInterval() { return 60 * 1000; } // Default poll interval (60 seconds)
};
