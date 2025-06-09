#pragma once

#include "I2CSensor.h"        // Base class for I2C sensor functionality in Meshtastic
#include "TelemetrySensor.h"  // Base class for Meshtastic telemetry sensors
#include "meshtastic/telemetry.pb.h" // Meshtastic protobuf definitions for telemetry data

// *** IMPORTANT: You MUST ensure the RAK12035_SoilMoisture library files are in your project's 'lib' folder. ***
// Using a direct, absolute path with quotes for the compiler to find the header.
#include "/workspace/firmware/lib/RAK12035_SoilMoisture/RAK12035_SoilMoisture.h"

// Define a placeholder for WB_IO2 if it's not defined elsewhere in your Meshtastic build.
// This pin is often used on RAK WisBlock boards to power sensor modules.
#ifndef WB_IO2
#define WB_IO2 17 // Common default for RAK4631 WisBlock IO2, verify for your board
#endif

// Renamed our adapter class to MeshtasticRAK12035Sensor to avoid conflict with the official library.
class MeshtasticRAK12035Sensor : public TelemetrySensor, public I2CSensor {
public:
    // Constructor: Takes the unique I2C address of the sensor (e.g., 0x21, 0x22, 0x23)
    // and the TwoWire instance (usually 'Wire' for I2C).
    MeshtasticRAK12035Sensor(uint8_t address, TwoWire &wire = Wire);

    // Meshtastic TelemetrySensor overrides:
    void init() override;     // Called to initialize the sensor module
    void setup() override;    // Performs hardware setup and library initialization
    int32_t runOnce() override; // Minimal implementation, as getMetrics does the work
    bool getMetrics(meshtastic_Telemetry *m) override; // Reads sensor data and populates telemetry
    bool hasSensor() override; // Checks if the sensor was successfully initialized

private:
    // Corrected class name from RAK12035_SoilMoisture to RAK12035 as defined in the library header.
    RAK12035 _rakSensor; // Instance of the official RAK12035 sensor object
    bool _isInitialized = false;      // Flag to track if the sensor was successfully initialized by its library
    bool _isDetected = false;         // Flag to track if the sensor is present on the I2C bus at its address
};
