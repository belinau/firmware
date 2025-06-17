/**
 * @file RAK12035_SoilMoisture.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Header file for Class RAK12035
 * @version 0.1
 * @date 2021-11-20
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef I2CSOILMOISTURESENSOR_H
#define I2CSOILMOISTURESENSOR_H

#include <Arduino.h>
#include <Wire.h>

#define SLAVE_I2C_ADDRESS_DEFAULT 0x20
//Soil Moisture Sensor Register Addresses
#define SOILMOISTURESENSOR_GET_CAPACITANCE 0x01	  // (r)   2 bytes
#define SOILMOISTURESENSOR_GET_I2C_ADDRESS 0x02	  // (r)   1 bytes
#define SOILMOISTURESENSOR_SET_I2C_ADDRESS 0x03	  // (w)   1 bytes
#define SOILMOISTURESENSOR_GET_VERSION 0x04		  // (r)   1 bytes
#define SOILMOISTURESENSOR_GET_TEMPERATURE 0x05	  // (r)   2 bytes
#define SOILMOISTURESENSOR_SET_SLEEP 0x06		  // (w)   1 bytes
#define SOILMOISTURESENSOR_SET_HUMIDITY_FULL 0x07 // (w)   2 bytes
#define SOILMOISTURESENSOR_SET_HUMIDITY_ZERO 0x08 // (w)   2 bytes
#define SOILMOISTURESENSOR_GET_HUMIDITY 0x09	  // (r)   1 bytes
#define SOILMOISTURESENSOR_GET_HUMIDITY_FULL 0x0A // (r)   2 bytes
#define SOILMOISTURESENSOR_GET_HUMIDITY_ZERO 0x0B // (r)   2 bytes

// Standard WisBlock IO pins used by RAK12023 for RAK12035
// These are included here for the library's internal use.
#ifndef WB_IO2
#define WB_IO2 17 // For 3V3_S power control
#endif
#ifndef WB_IO4
#define WB_IO4 21 // For sensor reset line
#endif
#ifndef WB_IO5
#define WB_IO5 26 // Only needed for VA boards (not directly controlled in this code)
#endif


class RAK12035
{
public:
	RAK12035(uint8_t addr = SLAVE_I2C_ADDRESS_DEFAULT);

	void setup(TwoWire &i2c_library = Wire);
	void begin(bool wait = true);
	bool get_sensor_version(uint8_t *version);
	bool get_sensor_capacitance(uint16_t *capacitance);
	bool get_sensor_moisture(uint8_t *moisture); // Original public method
	bool get_sensor_temperature(uint16_t *temperature);
	uint8_t get_sensor_addr(void);
	bool set_sensor_addr(uint8_t addr);
	bool set_i2c_addr(uint8_t addr);
	bool sensor_on(void);
	bool sensor_sleep(void);
	bool set_dry_cal(uint16_t zero_val);
	bool set_wet_cal(uint16_t hundred_val);
	bool get_dry_cal(uint16_t *zero_val);
	bool get_wet_cal(uint16_t *hundred_val);
	void reset(void);

	// NEW: Public methods for soft reset and raw moisture percentage calculation
	void soft_reset(); // Triggers a soft reset sequence using I2C commands/delays
	bool get_sensor_moisture_percentage(uint8_t *moisture_pct); // Gets the calculated percentage


private:
	int _sensorAddress = SLAVE_I2C_ADDRESS_DEFAULT;
	uint16_t _dry_cal = 590;
	uint16_t _wet_cal = 250;
	uint8_t _version = 0;
	bool read_rak12035(uint8_t reg, uint8_t *data, uint8_t length);
	bool write_rak12035(uint8_t reg, uint8_t *data, uint8_t length);
	TwoWire *_i2c_port = &Wire;

    // NEW: Static flag for global power management
    static bool _global_power_managed; // Declared in header
};
#endif