#include "RAK12035.h" // Now includes our renamed adapter class header (MeshtasticRAK12035Sensor)
#include "MeshService.h" // For Meshtastic LOG_INFO, LOG_WARN macros
#include <Wire.h>        // Required for I2C communication (Wire object)
#include <Arduino.h>     // For pinMode, digitalWrite, delay

// --- RAK WisBlock specific power pin. IMPORTANT: Ensure WB_IO2 is defined in your
// --- Meshtastic build environment (e.g., in variant files or configuration.h).
// --- If your board uses a different pin for sensor power, you'll need to
// --- change WB_IO2 to the correct GPIO number.
#ifndef WB_IO2
#define WB_IO2 17 // Common default for RAK4631 WisBlock IO2, verify for your board
#endif

// Constructor for our renamed adapter class
MeshtasticRAK12035Sensor::MeshtasticRAK12035Sensor(uint8_t address, TwoWire &wire) :
    TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "SoilSensor"), // Initialize Meshtastic TelemetrySensor base
    I2CSensor(address, wire), // Initialize Meshtastic I2CSensor base with the sensor's unique I2C address
    _rakSensor(address) // Corrected: Initialize the official RAK12035 object with its I2C address
{
    // The 'address' member inherited from I2CSensor (this->address) now holds the unique address (0x21, 0x22, or 0x23).
    // The _rakSensor object is also configured with this address internally.
}

// init() is required by the Meshtastic TelemetrySensor base class.
void MeshtasticRAK12035Sensor::init() {
    setup(); // Simply call setup() for initialization logic.
}

// setup() is where the sensor hardware is initialized and its presence/readiness is confirmed.
// This function now uses the official RAK12035 library's 'begin()' method.
void MeshtasticRAK12035Sensor::setup() {
    LOG_INFO("MeshtasticRAK12035Sensor::setup() called for address 0x%02X", this->address);

    // --- Step 1: Power ON the sensor module via WB_IO2 ---
    // This is crucial for many RAK WisBlock sensor modules.
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, HIGH); // Set WB_IO2 high to power on the sensor slot
    delay(100); // Give some time for the sensor to power up and stabilize

    // --- Step 2: Initialize the global I2C bus ---
    // Meshtastic usually handles this globally, but calling Wire.begin() here ensures it's ready.
    // Multiple calls to Wire.begin() are harmless.
    Wire.begin();
    delay(10); // Short delay after Wire.begin()

    // --- Step 3: Check for I2C device presence (initial acknowledge check) ---
    // This confirms the sensor is physically present and responds to its I2C address.
    Wire.beginTransmission(this->address);
    byte error = Wire.endTransmission();

    if (error == 0) { // Device acknowledged its address on I2C bus
        _isDetected = true; // Sensor is physically present
        LOG_INFO("Soil Sensor detected and acknowledged I2C address 0x%02X (Wire.endTransmission success)", this->address);

        // --- Step 4: Initialize the RAK12035 library instance ---
        // The begin() method in the RAK library handles the sensor's specific initialization,
        // including sending the AHT20 initialization command and internal calibration.
        // The 'false' argument means it won't block while waiting for the sensor to be ready.
        _rakSensor.begin(false); // Removed 'if' condition as begin() returns void
        
        // Assume initialization success if detected and begin() is called.
        // Actual status should be checked by attempting to read data, or by checking
        // the library's internal status flags if available, but for now we set it true
        // and rely on hasSensor() and getMetrics() to confirm success.
        _isInitialized = true; 

        // Optionally read version to confirm full functionality (as per RAK example)
        uint8_t version = 0;
        if (_rakSensor.get_sensor_version(&version)) {
            LOG_INFO("MeshtasticRAK12035Sensor::setup() sensor 0x%02X version: 0x%02X", this->address, version);
        } else {
            LOG_WARN("MeshtasticRAK12035Sensor::setup() sensor 0x%02X failed to get version after init.", this->address);
            _isInitialized = false; // Set to false if version read fails, indicates true initialization failure
        }

    } else { // Sensor not detected at its I2C address (Wire.endTransmission failed)
        _isDetected = false;
        _isInitialized = false; // Cannot initialize if not detected
        LOG_WARN("Soil Sensor NOT detected at I2C address 0x%02X, Wire.endTransmission returned error code: %d", this->address, error);
    }
}

// hasSensor() is required by the Meshtastic TelemetrySensor base class.
// Returns true if the sensor was both physically detected AND successfully initialized by its library in setup().
bool MeshtasticRAK12035Sensor::hasSensor() {
    return _isDetected && _isInitialized; // Both flags must be true for the sensor to be considered ready
}

// getMetrics() is where we read the data from the sensor and populate the Meshtastic telemetry packet.
// This function now uses the official RAK12035 library's data reading methods.
bool MeshtasticRAK12035Sensor::getMetrics(meshtastic_Telemetry *m) {
    if (!hasSensor()) { // Check if sensor is both detected AND initialized
        LOG_WARN("MeshtasticRAK12035Sensor::getMetrics() called for 0x%02X but sensor not available (detected=%d, initialized=%d). Skipping data read.",
                 this->address, _isDetected, _isInitialized);
        return false; // Cannot get metrics if sensor is not ready
    }

    LOG_INFO("MeshtasticRAK12035Sensor::getMetrics() called for address 0x%02X", this->address);

    uint16_t temperature_raw = 0;
    uint16_t capacitance_raw = 0;
    uint8_t moisture_pct = 0;
    bool success_readings = true; // Flag to track if all readings were successful

    // Attempt to read data using the official library's functions.
    // The library's functions handle the low-level I2C commands, polling, and status checks.
    if (!_rakSensor.get_sensor_temperature(&temperature_raw)) {
        LOG_WARN("MeshtasticRAK12035Sensor::getMetrics() failed to read temperature from 0x%02X", this->address);
        success_readings = false;
    }
    if (!_rakSensor.get_sensor_capacitance(&capacitance_raw)) {
        LOG_WARN("MeshtasticRAK12035Sensor::getMetrics() failed to read capacitance from 0x%02X", this->address);
        success_readings = false;
    }
    if (!_rakSensor.get_sensor_moisture(&moisture_pct)) {
        LOG_WARN("MeshtasticRAK12035Sensor::getMetrics() failed to read moisture percentage from 0x%02X", this->address);
        success_readings = false;
    }

    if (success_readings) { // Only proceed if all readings were successful
        float temp_c = temperature_raw / 10.0F;
        
        // Use the unique sensor's I2C address (this->address) to map the read data
        // to specific Meshtastic telemetry fields (voltage, current, IAQ).
        // This allows each of your three sensors to send distinct data.
        switch (this->address) {
            case 0x21: // For the RAK12035 at address 0x21 (mapped to Soil Sensor 1)
                m->variant.environment_metrics.voltage = (float)moisture_pct; // Soil moisture percentage in voltage field
                m->variant.environment_metrics.has_voltage = true;
                // Set overall temperature from the first sensor only (Meshtastic has one temp field).
                m->variant.environment_metrics.temperature = temp_c;
                m->variant.environment_metrics.has_temperature = true;
                LOG_INFO("MeshtasticRAK12035_0x21: T=%.2fC, M=%.0f%% (Cap=%u)", temp_c, (float)moisture_pct, capacitance_raw);
                break;
            case 0x22: // For the RAK12035 at address 0x22 (mapped to Soil Sensor 2)
                m->variant.environment_metrics.current = (float)moisture_pct; // Soil moisture percentage in current field
                m->variant.environment_metrics.has_current = true;
                LOG_INFO("MeshtasticRAK12035_0x22: T=%.2fC, M=%.0f%% (Cap=%u)", temp_c, (float)moisture_pct, capacitance_raw);
                break;
            case 0x23: // For the RAK12035 at address 0x23 (mapped to Soil Sensor 3)
                m->variant.environment_metrics.iaq = (uint32_t)moisture_pct; // Soil moisture percentage in IAQ field (as uint32_t)
                m->variant.environment_metrics.has_iaq = true;
                LOG_INFO("MeshtasticRAK12035_0x23: T=%.2fC, M=%.0f%% (Cap=%u)", temp_c, (float)moisture_pct, capacitance_raw);
                break;
            default:
                LOG_WARN("Soil Sensor: Unknown or unhandled sensor address 0x%02X in getMetrics, skipping data mapping.", this->address);
                return false;
        }
        return true; // Indicate success in getting and populating metrics
    }
    return false; // Indicate failure to read all data using library functions
}

// runOnce() is required by the Meshtastic TelemetrySensor base class.
// For this sensor, actual data reading is triggered by getMetrics() when the
// EnvironmentTelemetryModule's main loop decides it's time to send telemetry.
int32_t MeshtasticRAK12035Sensor::runOnce() {
    return 0; // Return 0 to indicate no specific action needed on its own thread loop.
              // The main Meshtastic loop will call getMetrics periodically based on EnvTelModule's scheduling.
}
