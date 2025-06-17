// RAK12035.h (Final Corrected Version)
#pragma once

#include "I2CSensor.h"
#include "TelemetrySensor.h"
#include "meshtastic/telemetry.pb.h"
#include "RAK12035_SoilMoisture.h" 

class MeshtasticRAK12035Sensor : public TelemetrySensor, public I2CSensor {
public:
    MeshtasticRAK12035Sensor(uint8_t address, TwoWire &wire = Wire);
    void init() override;
    void setup() override;
    int32_t runOnce() override;
    bool getMetrics(meshtastic_Telemetry *m) override;
    bool hasSensor() override;

    // Public methods to expose library functionality
    void reinitializeSensor(); // <-- MANJKAJOČA VRSTICA JE ZDAJ DODANA
    bool get_sensor_capacitance_raw(uint16_t *capacitance);
    bool get_sensor_moisture_raw_pct(uint8_t *moisture_pct);

private:
    RAK12035 _rakSensor;
    bool _isInitialized = false;
    bool _isDetected = false;
};