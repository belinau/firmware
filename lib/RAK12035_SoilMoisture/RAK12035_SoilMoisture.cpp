/*----------------------------------------------------------------------*
 * I2CSoilMoistureSensor.cpp - Arduino library for the Sensor version of*
 * I2C Soil Moisture Sensor version from Chrirp                         *
 * (https://github.com/Miceuz/i2c-moisture-sensor).                     *
 * *
 * Ingo Fischer 11Nov2015                                               *
 * https://github.com/Apollon77/I2CSoilMoistureSensor                   *
 * *
 * MIT license                                                          *
 *----------------------------------------------------------------------*/

#include "RAK12035_SoilMoisture.h"

#include <Wire.h>
#include <Arduino.h> // Required for pinMode, digitalWrite, delay etc.
#define i2cBegin _i2c_port->begin
#define i2cBeginTransmission _i2c_port->beginTransmission
#define i2cEndTransmission _i2c_port->endTransmission
#define i2cRequestFrom _i2c_port->requestFrom
#define i2cRead _i2c_port->read
#define i2cWrite _i2c_port->write

// NEW: Initialize static flag
bool RAK12035::_global_power_managed = false;

/*----------------------------------------------------------------------*
 * Constructor.                                                         *
 * Optionally set sensor I2C address if different from default          *
 *----------------------------------------------------------------------*/
RAK12035::RAK12035(uint8_t addr) : _sensorAddress(addr)
{
	// nothing to do ... Wire.begin needs to be put outside of class
}

void RAK12035::setup(TwoWire &i2c_library)
{
	_i2c_port = &i2c_library;
}

/*----------------------------------------------------------------------*
 * Initializes anything ... it does a reset.                            *
 * When used without parameter or parameter value is false then a       *
 * waiting time of at least 1 second is expected to give the sensor     *
 * some time to boot up.                                                *
 * Alternatively use true as parameter and the method waits for a       *
 * second and returns after that.                                       *
 *----------------------------------------------------------------------*/
void RAK12035::begin(bool wait)
{
    // NEW: Use static flag to control WB_IO2/WB_IO5/reset only once globally
    if (!_global_power_managed) {
        pinMode(WB_IO2, OUTPUT);
        digitalWrite(WB_IO2, HIGH);
        // Only needed for VA boards
        pinMode(WB_IO5, INPUT);

        // Reset the sensor
        reset(); // This calls the reset() function, which also uses WB_IO4
        
        _global_power_managed = true; // Set flag so it's only done once
    }

	delay(500); // Keep initial delay for sensor internal boot after external power-on

	time_t timeout = millis();
	uint8_t data;
	while (!get_sensor_version(&data))
	{
		if ((millis() - timeout) > 5000)
		{
			return;
		}
	}
	delay(500); // Delay after version check before reading calibration
	get_dry_cal(&_dry_cal);
	get_wet_cal(&_wet_cal);
}

// NEW: Implement soft_reset - calls library's sensor_on and begin
void RAK12035::soft_reset() {
    // This function will re-power and re-initialize the sensor using the library's built-in functions.
    // It will trigger WB_IO2/WB_IO4 via sensor_on() and begin(), which are now cooperative due to _global_power_managed.
    bool on_success = sensor_on();
    if (!on_success) {
        // Log a warning if sensor_on fails (requires a LOG_WARN macro in a context where it's defined)
    }
    delay(100); // Small delay after sensor_on()

    begin(false); // Re-run the begin sequence (version read, calib read)
}

// NEW: Implement get_sensor_moisture_percentage - original calculation logic
bool RAK12035::get_sensor_moisture_percentage(uint8_t *moisture_pct) {
    if (_version > 2)
	{
		return read_rak12035(SOILMOISTURESENSOR_GET_HUMIDITY, moisture_pct, 1);
	}
	else
	{
        uint16_t capacitance = 0;
        _i2c_port->setTimeout(5000);

        if (get_sensor_capacitance(&capacitance))
        {
            if (_dry_cal < _wet_cal)
            {
                if (capacitance <= _dry_cal) capacitance = _dry_cal;
                if (capacitance >= _wet_cal) capacitance = _wet_cal;
                moisture_pct[0] = (_wet_cal - capacitance) / ((_wet_cal - _dry_cal) / 100.0);
            }
            else
            {
                if (capacitance >= _dry_cal) capacitance = _dry_cal;
                if (capacitance <= _wet_cal) capacitance = _wet_cal;
                moisture_pct[0] = (_dry_cal - capacitance) / ((_dry_cal - _wet_cal) / 100.0);
            }
            if (moisture_pct[0] > 100)
            {
                moisture_pct[0] = 100;
            }
            return true;
        }
        return false;
	}
}


/**
 * @brief Get the sensor firmware version
 *
 * @param version The sensor firmware version
 * @return true I2C transmission success
 * @return false I2C transmission failed
 */
bool RAK12035::get_sensor_version(uint8_t *version)
{
	read_rak12035(SOILMOISTURESENSOR_GET_VERSION, &_version, 1);
	version[0] = _version;
	return read_rak12035(SOILMOISTURESENSOR_GET_VERSION, version, 1);
}

/**
 * @brief Get the sensor moisture value as capacitance value
 *
 * @param capacitance Variable to store value
 * @return true I2C transmission success
 * @return false I2C transmission failed
 */
bool RAK12035::get_sensor_capacitance(uint16_t *capacitance)
{
	uint8_t data[2] = {0};
	bool result = read_rak12035(SOILMOISTURESENSOR_GET_CAPACITANCE, data, 2);
	capacitance[0] = (((uint16_t)data[0]) << 8) | ((uint16_t)data[1]);
	return result;
}

// Original get_sensor_moisture - delegates to new get_sensor_moisture_percentage
bool RAK12035::get_sensor_moisture(uint8_t *moisture)
{
	return get_sensor_moisture_percentage(moisture);
}

/**
 * @brief Get the sensor temperature
 *
 * @param temperature as uint16_t value * 10
 * @return true I2C transmission success
 * @return false I2C transmission failed
 */
bool RAK12035::get_sensor_temperature(uint16_t *temperature)
{
	uint8_t data[2] = {0};
	bool result = read_rak12035(SOILMOISTURESENSOR_GET_TEMPERATURE, data, 2);
	temperature[0] = (((uint16_t)data[0]) << 8) | ((uint16_t)data[1]);
	return result;
}

/**
 * @brief Get the current I2C address from the sensor class
 *
 * @return the address the sensor class is using
 */
uint8_t RAK12035::get_sensor_addr(void)
{
	return _sensorAddress;
}

/**
 * @brief Set the new I2C address the sensor class will use.
 *
 * @param addr The new sensor address
 * @return false if the I2C address is invalid (only 1 to 127 is allowed)
 */
bool RAK12035::set_i2c_addr(uint8_t addr)
{
	if ((addr < 1) || (addr > 127))
	{
		return false;
	}
	_sensorAddress = addr;
	return true;
}

/**
 * @brief Set the new I2C address on the sensor. Requires a sensor reset after changing.
 *
 * @param addr The new sensor address
 * @return true I2C transmission success
 * @return false I2C transmission failed or if the I2C address is invalid (only 1 to 127 is allowed)
 */
bool RAK12035::set_sensor_addr(uint8_t addr)
{
	if ((addr < 1) || (addr > 127))
	{
		return false;
	}
	if (write_rak12035(SOILMOISTURESENSOR_SET_I2C_ADDRESS, &addr, 1))
	{
		_sensorAddress = addr;
		// Reset the sensor
		reset();
		return true;
	}
	return false;
}

/**
 * @brief Enable the power supply to the sensor
 *
 */
bool RAK12035::sensor_on(void)
{
	uint8_t data;
    // NEW: Use static flag to control WB_IO2/WB_IO4 only once globally
    if (!_global_power_managed) {
        digitalWrite(WB_IO2, HIGH);
        delay(250);

        digitalWrite(WB_IO4, LOW);
        delay(250);
        digitalWrite(WB_IO4, HIGH);
        _global_power_managed = true; // Set flag
    }
	// reset(); // Original comment - no change

	time_t timeout = millis();
	while (!get_sensor_version(&data))
	{
		if ((millis() - timeout) > 5000)
		{
			return false;
		}
		delay(250);
	}
	delay(500);
	return true;
}

/**
 * @brief Switch power supply of the sensor off
 *
 */
bool RAK12035::sensor_sleep(void)
{
	uint8_t tmp = 0;
	bool result = write_rak12035(SOILMOISTURESENSOR_SET_SLEEP, &tmp, 1);
	// Original library code keeps WB_IO2 control here.
	digitalWrite(WB_IO2, LOW);
	return result;
}

/**
 * @brief Set the dry value from the sensor calibration
 *
 * @param zero_val dry value
 */
bool RAK12035::set_dry_cal(uint16_t zero_val)
{
	uint8_t data[2];
	data[0] = zero_val >> 8;
	data[1] = zero_val;
	return write_rak12035(SOILMOISTURESENSOR_SET_HUMIDITY_ZERO, data, 2);
}

bool RAK12035::get_dry_cal(uint16_t *zero_val)
{
	uint8_t data[2] = {0};
	bool result = read_rak12035(SOILMOISTURESENSOR_GET_HUMIDITY_ZERO, data, 2);
	zero_val[0] = (((uint16_t)data[0]) << 8) | ((uint16_t)data[1]);
	_dry_cal = zero_val[0];
	return result;
}

/**
 * @brief Set the wet value from the sensor calibration
 *
 * @param hundred_val wet value
 */
bool RAK12035::set_wet_cal(uint16_t hundred_val)
{
	uint8_t data[2];
	data[0] = hundred_val >> 8;
	data[1] = hundred_val;
	return write_rak12035(SOILMOISTURESENSOR_SET_HUMIDITY_FULL, data, 2);
}

bool RAK12035::get_wet_cal(uint16_t *hundred_val)
{
	uint8_t data[2] = {0};
	bool result = read_rak12035(SOILMOISTURESENSOR_GET_HUMIDITY_FULL, data, 2);
	hundred_val[0] = (((uint16_t)data[0]) << 8) | ((uint16_t)data[1]);
	_wet_cal = hundred_val[0];
	return result;
}

/**
 * @brief Reset the sensor by pulling the reset line low.
 *
 */
void RAK12035::reset(void)
{
    // Original library code keeps WB_IO4 control here. It's called by begin() and set_sensor_addr().
    // It's not wrapped by _global_power_managed here, as it's a specific hardware reset that might
    // be needed even after _global_power_managed is true (e.g., if set_sensor_addr is called mid-run).
    // This maintains original library behavior for reset().
	pinMode(WB_IO4, OUTPUT);
	digitalWrite(WB_IO4, LOW);
	delay(500);
	digitalWrite(WB_IO4, HIGH);

	time_t timeout = millis();
	uint8_t data;
	while (!get_sensor_version(&data))
	{
		if ((millis() - timeout) > 5000)
		{
			return;
		}
		delay(250);
	}
	delay(500);
}

/**
 * @brief I2C read from sensor
 *
 * @param reg Sensor register to read
 * @param data Pointer to data buffer
 * @param length Number of bytes to read
 * @return true I2C transmission success
 * @return false I2C transmission failed
 */
bool RAK12035::read_rak12035(uint8_t reg, uint8_t *data, uint8_t length)
{
	i2cBeginTransmission(_sensorAddress);
	i2cWrite(reg);
	uint8_t result = i2cEndTransmission();
	if (result != 0)
	{
		return false;
	}
	delay(20);
	i2cRequestFrom(_sensorAddress, length);
	int i = 0;
	time_t timeout = millis();
	while (_i2c_port->available())
	{
		data[i++] = i2cRead();
		if ((millis() - timeout) > 1000)
		{
			break;
		}
		delay(10);
	}
	if (i != length)
	{
		return false;
	}
	return true;
}

/**
 * @brief I2C write to the sensor
 *
 * @param reg Register to write to
 * @param data Data to write
 * @return true I2C transmission success
 * @return false I2C transmission failed
 */
bool RAK12035::write_rak12035(uint8_t reg, uint8_t *data, uint8_t length)
{
	// *** CRITICAL FIX: Use _sensorAddress instead of hardcoded SLAVE_I2C_ADDRESS_DEFAULT ***
	_i2c_port->beginTransmission(_sensorAddress);
	_i2c_port->write(reg);
	for (int i = 0; i < length; i++)
	{
		_i2c_port->write(data[i]);
	}
	return (_i2c_port->endTransmission() == 0); // Return true if write was successful (0 means success)
}