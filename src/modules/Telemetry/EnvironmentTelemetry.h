#pragma once

// This macro controls whether the EnvironmentTelemetryModule is enabled.
// If not defined elsewhere, it defaults to 0 (disabled).
// The entire content of this header (except for this initial block)
// should be wrapped by this #ifndef/#endif pair.
#ifndef ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE
#define ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE 0

#include "../mesh/generated/meshtastic/telemetry.pb.h" // Protobuf definitions for telemetry messages
#include "NodeDB.h"          // Node database interface
#include "ProtobufModule.h"  // Base class for modules handling protobuf messages
#include <OLEDDisplay.h>     // Library for OLED display functionality
#include <OLEDDisplayUi.h>   // Library for OLED display UI management

// Forward declaration of our renamed sensor adapter class.
// This is necessary because EnvironmentTelemetryModule uses MeshtasticRAK12035Sensor,
// but the full definition of MeshtasticRAK12035Sensor might come later in compilation units.
class MeshtasticRAK12035Sensor;

/**
 * @brief Manages environment telemetry data, including reading from various sensors
 * and sending updates to the mesh or connected phone.
 *
 * This module extends ProtobufModule to handle Meshtastic Telemetry messages
 * and runs as a separate OS thread for periodic operations.
 */
class EnvironmentTelemetryModule : private concurrency::OSThread, public ProtobufModule<meshtastic_Telemetry>
{
public:
    // Constructor declaration. Declared public so it can be instantiated externally.
    // The definition of this constructor is in EnvironmentTelemetry.cpp.
    EnvironmentTelemetryModule(); 

    // Observer for node status updates.
    CallbackObserver<EnvironmentTelemetryModule, const meshtastic::Status *> nodeStatusObserver =
        CallbackObserver<EnvironmentTelemetryModule, const meshtastic::Status *>(this,
                                                                                 &EnvironmentTelemetryModule::handleStatusUpdate);

public: // Public interface of the module
    /**
     * @brief Indicates if the module wants a UI frame to be drawn on the screen.
     * @return True if a UI frame should be drawn, false otherwise.
     */
    virtual bool wantUIFrame() override;

#if !HAS_SCREEN // Conditional compilation for devices without a physical screen
    /**
     * @brief Draws the environment telemetry frame on the given display.
     * @param display Pointer to the OLEDDisplay object.
     * @param state Pointer to the OLEDDisplayUiState object.
     * @param x X-coordinate for drawing.
     * @param y Y-coordinate for drawing.
     */
    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
#else // For devices with a screen, override the base class method
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;
#endif

protected: // Protected methods for internal module operation
    /**
     * @brief Handles an incoming Meshtastic MeshPacket containing telemetry data.
     * @param mp The received MeshPacket.
     * @param p Pointer to the decoded Telemetry protobuf message.
     * @return True if the message was handled and no other handlers should process it, false otherwise.
     */
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *p) override;

    /**
     * @brief Called periodically by the OSThread framework.
     * @return The suggested delay in milliseconds until the next call.
     */
    virtual int32_t runOnce() override;

    /**
     * @brief Gathers current environment telemetry data from sensors.
     * @param m Pointer to the meshtastic_Telemetry struct to populate.
     * @return True if valid data was obtained, false otherwise.
     */
    bool getEnvironmentTelemetry(meshtastic_Telemetry *m);

    /**
     * @brief Allocates a reply packet for an incoming request.
     * @return A pointer to a newly allocated Meshtastic MeshPacket, or NULL if no reply is needed.
     */
    virtual meshtastic_MeshPacket *allocReply() override;

    /**
     * @brief Sends the current telemetry data to the mesh or a connected phone.
     * @param dest The destination node number (e.g., NODENUM_BROADCAST).
     * @param phoneOnly If true, sends only to the phone and not to the mesh.
     * @return True if the telemetry packet was successfully sent, false otherwise.
     */
    bool sendTelemetry(NodeNum dest = NODENUM_BROADCAST, bool phoneOnly = false);

    /**
     * @brief Handles administrative messages directed to this module.
     * @param mp The received MeshPacket.
     * @param request Pointer to the decoded AdminMessage request.
     * @param response Pointer to the AdminMessage response to populate.
     * @return Result indicating if the message was handled.
     */
    virtual AdminMessageHandleResult handleAdminMessageForModule(const meshtastic_MeshPacket &mp,
                                                                 meshtastic_AdminMessage *request,
                                                                 meshtastic_AdminMessage *response) override;

private: // Private member variables
    bool firstTime = true; // Flag for initial setup
    meshtastic_MeshPacket *lastMeasurementPacket = nullptr; // Stores the last sent/received measurement packet
    uint32_t sendToPhoneIntervalMs; // Interval for sending data to the phone
    uint32_t lastSentToMesh;        // Timestamp of the last data sent to the mesh
    uint32_t lastSentToPhone;       // Timestamp of the last data sent to the phone
    uint32_t sensor_read_error_count; // Counter for sensor read errors
    bool sleepOnNextExecution = false; // Flag to trigger deep sleep after next execution
};

// This #endif correctly closes the #ifndef ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE
#endif // ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE
