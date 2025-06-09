#pragma once

// NOTE: Removed the #if defined(ARCH_NRF52) wrapper which was causing the build failure.

#include "I2CSensor.h"
#include "TelemetrySensor.h"
#include "meshtastic/telemetry.pb.h"
#include <Wire.h> // Include Wire.h for TwoWire reference

#define RAK12035_I2C_ADDR 0x38 // This is the default AHT20 address, but your sensors use 0x21, 0x22, 0x23 directly.
                               // This constant is no longer used in the class logic, but kept for reference.

struct RAK12035_Data {
    float temperature;
    float humidity;
};

class RAK12035 : public TelemetrySensor, public I2CSensor {
public:
    // Constructor now accepts the unique I2C address for each RAK12035 sensor (e.g., 0x21, 0x22, 0x23)
    RAK12035(uint8_t address, TwoWire &wire = Wire);

    void init() override;
    void setup() override;
    int32_t runOnce() override;
    
    bool getMetrics(meshtastic_Telemetry *m) override;
    bool hasSensor() override;

private:
    // Private function to read raw data from the sensor
    bool read(RAK12035_Data &data);

    bool isDetected = false; // Flag to indicate if the sensor was detected during setup
};
