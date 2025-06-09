#pragma once

// NOTE: Removed the #if defined(ARCH_NRF52) wrapper which was causing the build failure.

#include "I2CSensor.h"
#include "TelemetrySensor.h"
#include "meshtastic/telemetry.pb.h"

#define RAK12035_I2C_ADDR 0x38

struct RAK12035_Data {
    float temperature;
    float humidity;
};

class RAK12035 : public TelemetrySensor, public I2CSensor {
public:
    RAK12035(uint8_t address, TwoWire &wire = Wire); // It now takes an I2C address

    void init() override;
    void setup() override;
    int32_t runOnce() override;
    
    bool getMetrics(meshtastic_Telemetry *m) override;
    bool hasSensor() override;

private:
    bool read(RAK12035_Data &data);

    bool isDetected = false;
// uint8_t _mux_channel; // No longer needed
};