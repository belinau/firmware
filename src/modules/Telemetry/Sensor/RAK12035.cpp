#include "RAK12035.h"
#include "MeshService.h"

// The constructor now takes an address and passes it to the I2CSensor base class
RAK12035::RAK12035(uint8_t address, TwoWire &wire) :
    TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "SoilSensor"),
    I2CSensor(address, wire) // Pass the address to the base class
{
    // The member variable 'address' is now handled by the I2CSensor base class
}

// init() is required by the base class.
void RAK12035::init() {
    setup(); // We can just have it call setup()
}

// setup() is also required by the base class.
void RAK12035::setup() {
    // We just need to check if the sensor is present at the address we were given
    if (I2CSensor::isPresent()) { // isPresent() uses the address from the base class
        isDetected = true;
        LOG_INFO("Soil Sensor found at address 0x%02X", this->address);
    } else {
        isDetected = false;
        LOG_WARN("Soil Sensor not found at address 0x%02X", this->address);
    }
}

// hasSensor() is required by the base class.
bool RAK12035::hasSensor() {
    return isDetected;
}

// getMetrics() is where we read the data and put it into the telemetry packet
bool RAK12035::getMetrics(meshtastic_Telemetry *m) {
    if (!hasSensor()) return false;

    RAK12035_Data data;
    if (read(data)) {
        // Use the I2C address to decide which telemetry field to use
        // This prevents the sensors from overwriting each other's data
        switch (this->address) {
            case 0x21: // Sensor 1
                // We'll put soil moisture in the 'voltage' field
                m->variant.environment_metrics.voltage = data.humidity;
                m->variant.environment_metrics.has_voltage = true;
                break;
            case 0x22: // Sensor 2
                // We'll put soil moisture in the 'current' field
                m->variant.environment_metrics.current = data.humidity;
                m->variant.environment_metrics.has_current = true;
                break;
            case 0x23: // Sensor 3
                // We'll put soil moisture in the 'iaq' (air quality) field
                m->variant.environment_metrics.iaq = (uint32_t)data.humidity;
                m->variant.environment_metrics.has_iaq = true;
                break;
        }
        // We can also send temperature from one of the sensors
        if (this->address == 0x21) {
             m->variant.environment_metrics.temperature = data.temperature;
             m->variant.environment_metrics.has_temperature = true;
        }
        return true;
    }
    return false;
}

// runOnce() is required but we don't need it to do anything for this sensor
int32_t RAK12035::runOnce() {
    return 0;
}

// A private function to read the data from the hardware
bool RAK12035::read(RAK12035_Data &data) {
    // This uses the readBytes function from your I2CSensor base class
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    if (!writeBytes(0x71, cmd, 3)) {
      LOG_WARN("Soil Sensor: Failed to send command to 0x%02X", this->address);
      return false;
    }
    delay(80); // Wait for measurement

    uint8_t raw_data[6];
    if (!readBytes(0x71, raw_data, 6)) {
        LOG_WARN("Soil Sensor: Failed to read data from 0x%02X", this->address);
        return false;
    }
    
    // Convert raw data to humidity % and temperature
    uint32_t raw_humidity = ((uint32_t)raw_data[1] << 12) | ((uint32_t)raw_data[2] << 4) | ((raw_data[3] >> 4) & 0x0F);
    data.humidity = (raw_humidity / (float)0x100000) * 100.0;

    uint32_t raw_temp = (((uint32_t)raw_data[3] & 0x0F) << 16) | ((uint32_t)raw_data[4] << 8) | raw_data[5];
    data.temperature = ((raw_temp / (float)0x100000) * 200.0) - 50.0;
    
    return true;
}