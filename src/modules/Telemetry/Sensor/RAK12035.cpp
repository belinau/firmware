// RAK12035.cpp (v6 - Pre-Read Reset Test)
#include "RAK12035.h"
#include "MeshService.h" 
#include <Wire.h>        
#include <Arduino.h>     

MeshtasticRAK12035Sensor::MeshtasticRAK12035Sensor(uint8_t address, TwoWire &wire) :
    TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "SoilSensor"),
    I2CSensor(address, wire),
    _rakSensor(address)
{
}

void MeshtasticRAK12035Sensor::init() {
    setup();
}

void MeshtasticRAK12035Sensor::setup() {
    LOG_INFO("MeshtasticRAK12035Sensor::setup() called for address 0x%02X", this->address);
    _rakSensor.setup(this->wire); 

    this->wire.beginTransmission(this->address);
    byte error = this->wire.endTransmission();
    
    if (error == 0) {
        _isDetected = true;
        LOG_INFO("MeshtasticRAK12035Sensor: Soil Sensor detected at I2C address 0x%02X", this->address);
        
        _rakSensor.begin(false); 
        
        uint8_t version = 0;
        if (_rakSensor.get_sensor_version(&version)) {
            LOG_INFO("MeshtasticRAK12035Sensor::setup() sensor 0x%02X version: 0x%02X", this->address, version);
        } else {
            LOG_WARN("MeshtasticRAK12035Sensor::setup() sensor 0x%02X failed to get version.", this->address);
        }
        
        uint16_t dry_cal_val, wet_cal_val;
        if (_rakSensor.get_dry_cal(&dry_cal_val) && _rakSensor.get_wet_cal(&wet_cal_val)) {
            LOG_INFO("RAK12035 sensor 0x%02X retrieved dry_cal: %u, wet_cal: %u", this->address, dry_cal_val, wet_cal_val);
            _isInitialized = true; 
        } else {
            LOG_WARN("MeshtasticRAK12035Sensor: 0x%02X failed to retrieve calibration values.", this->address);
            _isInitialized = false;
        }
    } else {
        _isDetected = false;
        _isInitialized = false;
        LOG_WARN("Soil Sensor NOT detected at I2C address 0x%02X, error: %d", this->address, error);
    }
}

bool MeshtasticRAK12035Sensor::hasSensor() {
    return _isDetected && _isInitialized;
}

bool MeshtasticRAK12035Sensor::getMetrics(meshtastic_Telemetry *m) {
    if (!hasSensor()) {
        return false;
    }

    // *** TEST NOVE HIPOTEZE ***
    // Če je to senzor na problematičnem priklopu (trenutno z naslovom 0x21),
    // pred branjem sprožimo hardverski reset.
    if (this->address == 0x21) {
        LOG_INFO("RAK12035: Applying pre-read HARDWARE RESET for problematic sensor 0x21");
        _rakSensor.reset(); // Klic hardverskega reseta preko WB_IO4
        delay(200);         // Kratka pavza, da se senzor "umiri" po resetu
    }

    uint16_t temperature_raw = 0;
    uint16_t capacitance_raw = 0;
    uint8_t moisture_pct = 0;

    bool temp_ok = _rakSensor.get_sensor_temperature(&temperature_raw);
    bool cap_ok = _rakSensor.get_sensor_capacitance(&capacitance_raw);
    bool moisture_calc_ok = _rakSensor.get_sensor_moisture_percentage(&moisture_pct);

    if (!temp_ok || !cap_ok || !moisture_calc_ok) {
        LOG_WARN("MeshtasticRAK12035Sensor::getMetrics() failed to read data from 0x%02X.", this->address);
        return false;
    }

    float temp_c = temperature_raw / 10.0F;
    LOG_INFO("MeshtasticRAK12035_0x%02X: T=%.2fC, M=%u%% (Cap=%u)", this->address, temp_c, moisture_pct, capacitance_raw);

    m->variant.environment_metrics.temperature = temp_c;
    m->variant.environment_metrics.has_temperature = true;

    switch (this->address) {
        case 0x21:
            m->variant.environment_metrics.voltage = (float)moisture_pct;
            m->variant.environment_metrics.has_voltage = true;
            break;
        case 0x22:
            m->variant.environment_metrics.current = (float)moisture_pct;
            m->variant.environment_metrics.has_current = true;
            break;
        case 0x23:
            m->variant.environment_metrics.iaq = (uint32_t)moisture_pct;
            m->variant.environment_metrics.has_iaq = true;
            break;
        default:
            LOG_WARN("Soil Sensor: Unknown address 0x%02X in getMetrics.", this->address);
            return false;
    }
    return true;
}

int32_t MeshtasticRAK12035Sensor::runOnce() {
    return 0;
}

void MeshtasticRAK12035Sensor::reinitializeSensor() {
    LOG_WARN("MeshtasticRAK12035Sensor: Re-initializing sensor 0x%02X", this->address);
    _isInitialized = false;
    _isDetected = false;
    setup();
}

bool MeshtasticRAK12035Sensor::get_sensor_capacitance_raw(uint16_t *capacitance) {
    if (!hasSensor()) return false;
    return _rakSensor.get_sensor_capacitance(capacitance);
}

bool MeshtasticRAK12035Sensor::get_sensor_moisture_raw_pct(uint8_t *moisture_pct) {
    if (!hasSensor()) return false;
    return _rakSensor.get_sensor_moisture_percentage(moisture_pct);
}