    #pragma once

    // Only compile for nRF52-based boards like the RAK4631
    #if defined(ARCH_NRF52) && !defined(ARDUINO_NANO_RP2040_CONNECT)

    #include "../I2CSensor.h"
    #include "../mesh/generated/meshtastic/telemetry.pb.h" // Needed for meshtastic_Telemetry struct

    // RAK12035 default I2C Address and Registers
    #define RAK12035_I2C_ADDR 0x78
    #define RAK12035_REG_DATA 0x00 // Register to read both temp & humidity

    // A structure to hold the sensor readings
    struct RAK12035_Data {
    float temperature;
    float humidity;
    };

    class RAK12035 : public I2CSensor {
    public:
    // Constructor
    RAK12035(TwoWire &wire, uint8_t variant) : I2CSensor(RAK12035_I2C_ADDR, wire, variant) {}

    // Standard sensor functions
    void init() override;
    bool read(RAK12035_Data &data); // Custom read function for this sensor
    bool getMetrics(meshtastic_Telemetry *m); // New function to populate telemetry struct
    };

    #endif
    