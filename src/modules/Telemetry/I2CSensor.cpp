    #include "I2CSensor.h"
    #include "meshUtils.h" // Assuming this provides LOG_DEBUG, LOG_WARNING etc.

    // Checks if the sensor responds on the I2C bus at its specified address
    bool I2CSensor::isPresent() {
        wire.beginTransmission(address);
        return wire.endTransmission() == 0;
    }

    // Reads a specified number of bytes from a register
    bool I2CSensor::readBytes(uint8_t reg, uint8_t *buffer, size_t len) {
        wire.beginTransmission(address);
        wire.write(reg);
        if (wire.endTransmission(false) != 0) { // Send restart, do not release bus
            LOG_WARN("I2C readBytes: Failed to begin transmission or write register.");
            return false;
        }

        uint8_t bytesRead = wire.requestFrom(address, len);
        if (bytesRead != len) {
            LOG_WARN("I2C readBytes: Expected %d bytes, got %d.", len, bytesRead);
            return false;
        }

        for (size_t i = 0; i < len; i++) {
            buffer[i] = wire.read();
        }
        return true;
    }

    // Writes a specified number of bytes to a register
    bool I2CSensor::writeBytes(uint8_t reg, uint8_t *buffer, size_t len) {
        wire.beginTransmission(address);
        wire.write(reg);
        for (size_t i = 0; i < len; i++) {
            wire.write(buffer[i]);
        }
        return wire.endTransmission() == 0;
    }
    