#pragma once

// Only compile for nRF52-based boards like the RAK4631
#if defined(ARCH_NRF52) && !defined(ARDUINO_NANO_RP2040_CONNECT)

#include "../I2CSensor.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"

#define RAK12035_I2C_ADDR 0x78
#define RAK12035_REG_DATA 0x00

struct RAK12035_Data {
    float temperature;
    float humidity;
};

class RAK12035 : public I2CSensor {
public:
    RAK12035(TwoWire &wire, uint8_t variant) : I2CSensor(RAK12035_I2C_ADDR, wire, variant) {}

    void init() override;
    bool read(RAK12035_Data &data);
    bool getMetrics(meshtastic_Telemetry *m);
    
    // ADDED: Handle admin messages
    AdminMessageHandleResult handleAdminMessage(const meshtastic_MeshPacket &mp, 
                                               meshtastic_AdminMessage *request,
                                               meshtastic_AdminMessage *response) {
        return AdminMessageHandleResult::NOT_HANDLED;
    }
};

#endif