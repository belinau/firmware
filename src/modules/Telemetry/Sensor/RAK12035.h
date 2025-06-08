#include "Sensor/RAK12035.h"
#include "meshtastic/util.h"

#if defined(ARCH_NRF52) && !defined(ARDUINO_NANO_RP2040_CONNECT)

/**
* @brief Initialize the sensor by checking for its presence on the I2C bus.
*/
void RAK12035::init() {
if (I2CSensor::isPresent()) {
LOG_INFO("RAK12035 Soil Moisture Sensor found.");
isDetected = true;
} else {
LOG_WARNING("RAK12035 Soil Moisture Sensor not found on this MUX channel!");
isDetected = false;
}
}

/**
* @brief Read the temperature and humidity data from the sensor.
*
* @param data A reference to a RAK12035_Data struct to store the results.
* @return True if the read was successful, false otherwise.
*/
bool RAK12035::read(RAK12035_Data &data) {
if (!isDetected) {
return false;
}

uint8_t sensorData[4];
if (readBytes(RAK12035_REG_DATA, sensorData, 4)) {
// Data is returned as two 16-bit unsigned integers (big-endian)
uint16_t hum_raw = ((uint16_t)sensorData[0] << 8) | sensorData[1];
uint16_t temp_raw = ((uint16_t)sensorData[2] << 8) | sensorData[3];

// Convert to floating point values per the datasheet (value / 10.0)
data.humidity = (float)hum_raw / 10.0f;
data.temperature = (float)temp_raw / 10.0f;

LOG_DEBUG("RAK12035 Reading: Temp=%.2f C, Hum=%.2f %%RH", data.temperature, data.humidity);
return true;
} else {
LOG_WARNING("Failed to read from RAK12035 sensor.");
return false;
}
}

/**
* @brief Populate the meshtastic_Telemetry struct with RAK12035 sensor metrics.
* This function ensures that temperature and relative humidity are only set if
* they haven't been already populated by a higher-priority sensor.
* @param m A pointer to the meshtastic_Telemetry struct to populate.
* @return True if metrics were successfully read and populated, false otherwise.
*/
bool RAK12035::getMetrics(meshtastic_Telemetry *m) { // ADD THIS FUNCTION
    RAK12035_Data data;
    if (read(data)) {
        // Only update temperature if it's not already set by another sensor
        if (!m->variant.environment_metrics.has_temperature) {
            m->variant.environment_metrics.temperature = data.temperature;
            m->variant.environment_metrics.has_temperature = true;
        }
        // Only update relative_humidity if it's not already set by another sensor
        if (!m->variant.environment_metrics.has_relative_humidity) {
            m->variant.environment_metrics.relative_humidity = data.humidity;
            m->variant.environment_metrics.has_relative_humidity = true;
        }
        LOG_INFO("RAK12035::getMetrics - Temp: %.2f, Hum: %.2f", data.temperature, data.humidity);
        return true;
    }
    return false;
}

#endif
