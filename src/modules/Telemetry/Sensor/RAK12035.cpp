#include "RAK12035.h"
#include "MeshService.h" // For LOG_INFO, LOG_WARN macros
#include <Wire.h>        // Explicitly include Wire.h for I2C communication
#include <Arduino.h>     // For pinMode, digitalWrite, delay

// --- RAK WisBlock specific power pin. IMPORTANT: Ensure WB_IO2 is defined in your
// --- Meshtastic build environment (e.g., in variant files or configuration.h).
// --- If your board uses a different pin for sensor power, you'll need to
// --- change WB_IO2 to the correct GPIO number.
#ifndef WB_IO2
#define WB_IO2 17 // Common default for RAK4631 WisBlock IO2
#endif

// The constructor now takes the unique I2C address of the sensor (0x21, 0x22, or 0x23)
// and passes it directly to the I2CSensor base class.
RAK12035::RAK12035(uint8_t address, TwoWire &wire) :
    TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "SoilSensor"), // Initialize base class
    I2CSensor(address, wire) // Pass the unique sensor address to the I2CSensor base class
{
    // The 'address' member variable inherited from I2CSensor now holds the unique address (0x21, 0x22, or 0x23)
}

// init() is required by the base class.
void RAK12035::init() {
    setup(); // Simply call setup() for initialization logic
}

// setup() is where the sensor is initialized and detected.
// This function now includes explicit power-on and a robust I2C probe.
// It uses the global 'Wire' object for I2C communication to resolve the '_wire' scope error.
void RAK12035::setup() {
    LOG_INFO("RAK12035::setup() called for address 0x%02X", this->address);

    // --- Step 1: Power ON the sensor module via WB_IO2 (if applicable for your board) ---
    // This is crucial for many RAK WisBlock sensor modules.
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, HIGH); // Set WB_IO2 high to power on the sensor slot
    delay(100); // Give some time for the sensor to power up and stabilize

    // --- Step 2: Initialize the I2C bus ---
    // Ensure the I2C bus is initialized before attempting communication.
    // This calls Wire.begin() on the global I2C object, which is typically done once globally.
    // Including it here ensures it's done if not handled elsewhere, and multiple calls are harmless.
    Wire.begin(); // FIX: Use global 'Wire' object directly
    delay(10); // Short delay after Wire.begin()

    // --- Step 3: Perform a robust I2C device presence check ---
    // This explicitly attempts to start communication with the sensor's address
    // and checks if it acknowledges.
    Wire.beginTransmission(this->address); // FIX: Use global 'Wire' object directly
    byte error = Wire.endTransmission();   // FIX: Use global 'Wire' object directly

    if (error == 0) { // Error code 0 means success (device acknowledged its address)
        isDetected = true;
        LOG_INFO("Soil Sensor detected and acknowledged I2C address 0x%02X (Wire.endTransmission success)", this->address);
        // Optional: Perform a sensor-specific check if needed, e.g., reading a version register.
    } else {
        isDetected = false;
        LOG_WARN("Soil Sensor NOT detected at I2C address 0x%02X, Wire.endTransmission returned error code: %d", this->address, error);
        // Common I2C error codes from Wire.endTransmission():
        // 0: success
        // 1: data too long to fit in transmit buffer
        // 2: received NACK on transmit of address (device not found at address)
        // 3: received NACK on transmit of data
        // 4: other error
        // 5: timeout
    }
}

// hasSensor() is required by the base class.
// Returns true if the sensor was detected during setup.
bool RAK12035::hasSensor() {
    return isDetected;
}

// getMetrics() is where we read the data from the sensor and populate the telemetry packet.
bool RAK12035::getMetrics(meshtastic_Telemetry *m) {
    if (!hasSensor()) return false; // If sensor not detected, don't try to read

    LOG_INFO("RAK12035::getMetrics() called for address 0x%02X", this->address); // Debug log

    RAK12035_Data data;
    if (read(data)) { // Attempt to read data from the sensor
        // Use the unique sensor's I2C address (this->address) to decide
        // which specific telemetry field to populate. This ensures each
        // soil sensor populates a distinct field in the Meshtastic telemetry packet.
        switch (this->address) {
            case 0x21: // For the RAK12035 at address 0x21 (Soil Sensor 1)
                // Map soil moisture (humidity from RAK12035_Data) to the 'voltage' field.
                m->variant.environment_metrics.voltage = data.humidity;
                m->variant.environment_metrics.has_voltage = true;
                // Also, set the overall temperature from the first soil sensor.
                // Note: Only one temperature field is available in meshtastic_Telemetry,
                // so subsequent temperature sensors would overwrite this.
                m->variant.environment_metrics.temperature = data.temperature;
                m->variant.environment_metrics.has_temperature = true;
                break;
            case 0x22: // For the RAK12035 at address 0x22 (Soil Sensor 2)
                // Map soil moisture to the 'current' field.
                m->variant.environment_metrics.current = data.humidity;
                m->variant.environment_metrics.has_current = true;
                break;
            case 0x23: // For the RAK12035 at address 0x23 (Soil Sensor 3)
                // Map soil moisture to the 'iaq' (Indoor Air Quality) field.
                // Note: IAQ is a uint32_t, so we cast the float humidity.
                m->variant.environment_metrics.iaq = (uint32_t)data.humidity;
                m->variant.environment_metrics.has_iaq = true;
                break;
            default:
                // Log a warning if an unexpected sensor address tries to get metrics
                LOG_WARN("Soil Sensor: Unknown or unhandled sensor address 0x%02X in getMetrics", this->address);
                return false; // Indicate failure to get metrics for this unknown address
        }
        return true; // Indicate success in getting and populating metrics
    }
    return false; // Indicate failure to read data from sensor
}

// runOnce() is required by the base class but doesn't need to do anything for this sensor
// as data reading is triggered by getMetrics().
int32_t RAK12035::runOnce() {
    return 0; // Return 0 to indicate no specific action needed on its own thread loop
}

// A private function to handle the low-level I2C communication and data parsing.
// This function sends commands to and reads data from the RAK12035 sensor at its specific I2C address.
bool RAK12035::read(RAK12035_Data &data) {
    LOG_INFO("RAK12035::read() called for address 0x%02X", this->address); // Debug log

    // AHT20 (commonly used in RAK12035) command for measurement
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    
    // Send command to the sensor at its unique address (this->address, e.g., 0x21, 0x22, 0x23)
    if (!writeBytes(this->address, cmd, 3)) {
      LOG_WARN("Soil Sensor: Failed to send command to 0x%02X (writeBytes)", this->address);
      return false;
    }
    delay(80); // Wait for measurement to complete (AHT20 requires approx. 75ms)

    uint8_t raw_data[6]; // Buffer to store raw sensor data (6 bytes for AHT20)
    // Read data from the sensor at its unique address
    if (!readBytes(this->address, raw_data, 6)) {
        LOG_WARN("Soil Sensor: Failed to read data from 0x%02X (readBytes)", this->address);
        return false;
    }
    
    // Convert raw 24-bit data for humidity and temperature according to AHT20 datasheet.
    // Humidity conversion: raw data to percentage
    uint32_t raw_humidity = ((uint32_t)raw_data[1] << 12) | ((uint32_t)raw_data[2] << 4) | ((raw_data[3] >> 4) & 0x0F);
    data.humidity = (raw_humidity / (float)0x100000) * 100.0; // Max raw value 0x100000 = 1048576

    // Temperature conversion: raw data to Celsius
    uint32_t raw_temp = (((uint32_t)raw_data[3] & 0x0F) << 16) | ((uint32_t)raw_data[4] << 8) | raw_data[5];
    data.temperature = ((raw_temp / (float)0x100000) * 200.0) - 50.0; // Max raw value 0x100000 = 1048576
    
    LOG_INFO("RAK12035::read() success for 0x%02X: Temp=%.2f, Hum=%.2f", this->address, data.temperature, data.humidity);
    return true; // Indicate successful data read and parsing
}
