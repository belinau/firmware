/*----------------------------------------------------------------------*
 * I2CSoilMoistureSensor.cpp - Arduino library for the Sensor version of*
 * I2C Soil Moisture Sensor version from Chrirp                         *
 * (https://github.com/Miceuz/i2c-moisture-sensor).                     *
 *                                                                      *
 * Ingo Fischer 11Nov2015                                               *
 * https://github.com/Apollon77/I2CSoilMoistureSensor                   *
 *                                                                      *
 * MIT license                                                          *
 *----------------------------------------------------------------------*/

#include "RAK12035_SoilMoisture.h"

#include <Wire.h>
#define i2cBegin _i2c_port->begin
#define i2cBeginTransmission _i2c_port->beginTransmission
#define i2cEndTransmission _i2c_port->endTransmission
#define i2cRequestFrom _i2c_port->requestFrom
#define i2cRead _i2c_port->read
#define i2cWrite _i2c_port->write

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
	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);
	// Only needed for VA boards
	pinMode(WB_IO5, INPUT);

	// Reset the sensor
	reset();

	delay(500);

	time_t timeout = millis();
	uint8_t data;
	while (!get_sensor_version(&data))
	{
		if ((millis() - timeout) > 5000)
		{
			return;
		}
	}
	delay(500);
	get_dry_cal(&_dry_cal);
	get_wet_cal(&_wet_cal);
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

/**
 * @brief Get the sensor moisture value as percentage
 *
 * @param moisture Variable to store the value
 * @return true I2C transmission success
 * @return false I2C transmission failed
 */
bool RAK12035::get_sensor_moisture(uint8_t *moisture)
{
	if (_version > 2)
	{
		bool result = read_rak12035(SOILMOISTURESENSOR_GET_HUMIDITY, moisture, 1);
		return result;
	}
	else
	{
		uint16_t capacitance = 0;
		_i2c_port->setTimeout(5000);

		if (get_sensor_capacitance(&capacitance))
		{
			if (_dry_cal < _wet_cal)
			{
				if (capacitance <= _dry_cal)
				{
					capacitance = _dry_cal;
				}
				if (capacitance >= _wet_cal)
				{
					capacitance = _wet_cal;
				}

				moisture[0] = (_wet_cal - capacitance) / ((_wet_cal - _dry_cal) / 100.0);
			}
			else
			{
				if (capacitance >= _dry_cal)
				{
					capacitance = _dry_cal;
				}
				if (capacitance <= _wet_cal)
				{
					capacitance = _wet_cal;
				}

				moisture[0] = (_dry_cal - capacitance) / ((_dry_cal - _wet_cal) / 100.0);
			}
			if (moisture[0] > 100)
			{
				moisture[0] = 100;
			}
			return true;
		}
		return false;
	}
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
	digitalWrite(WB_IO2, HIGH);
	delay(250);

	digitalWrite(WB_IO4, LOW);
	delay(250);
	digitalWrite(WB_IO4, HIGH);
	// reset();

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
	digitalWrite(WB_IO2, LOW);
	return result;
	// return write_rak12035(SOILMOISTURESENSOR_SET_SLEEP, &tmp, 1);
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
	i2cWrite(reg);						   // sends five bytes
	uint8_t result = i2cEndTransmission(); // stop transmitting
	if (result != 0)
	{
		return false;
	}
	delay(20);
	i2cRequestFrom(_sensorAddress, length);
	int i = 0;
	time_t timeout = millis();
	while (_i2c_port->available()) // slave may send less than requested
	{
		data[i++] = i2cRead(); // receive a byte as a proper uint8_t
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
	_i2c_port->beginTransmission(_sensorAddress); // Use the current sensor address
	_i2c_port->write(reg); // sends five bytes
	for (int i = 0; i < length; i++)
	{
		_i2c_port->write(data[i]);
	}
	// return true; // Old problematic line, always returned true
	return (_i2c_port->endTransmission() == 0); // Return true if write was successful (0 means success)
}
