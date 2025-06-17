// EnvironmentTelemetry.cpp (Final, Complete & Verified - Part 1/2)
#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "Default.h"
#include "EnvironmentTelemetry.h" 
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "Router.h"
#include "UnitConversions.h"
#include "main.h"
#include "power.h"
#include "sleep.h"
#include "target_specific.h"
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>
#include <Arduino.h>


#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL
// Sensors
#include "Sensor/CGRadSensSensor.h"
#include "Sensor/RCWL9620Sensor.h"
#include "Sensor/nullSensor.h"
#include "Sensor/RAK12035.h"

#ifndef WB_IO2
#define WB_IO2 17
#endif
#ifndef WB_IO4
#define WB_IO4 21
#endif


#if __has_include(<Adafruit_AHTX0.h>)
#include "Sensor/AHT10.h"
AHT10Sensor aht10Sensor;
#else
NullSensor aht10Sensor;
#endif
#if __has_include(<Adafruit_BME280.h>)
#include "Sensor/BME280Sensor.h"
BME280Sensor bme280Sensor;
#else
NullSensor bmp280Sensor;
#endif
#if __has_include(<Adafruit_BMP085.h>)
#include "Sensor/BMP085Sensor.h"
BMP085Sensor bmp085Sensor;
#else
NullSensor bmp085Sensor;
#endif
#if __has_include(<Adafruit_BMP280.h>)
#include "Sensor/BMP280Sensor.h"
BME280Sensor bmp280Sensor;
#else
NullSensor bme280Sensor;
#endif
#if __has_include(<Adafruit_LTR390.h>)
#include "Sensor/LTR390UVSensor.h"
LTR390UVSensor ltr390uvSensor;
#else
NullSensor ltr390uvSensor;
#endif
#if __has_include(<bsec2.h>)
#include "Sensor/BME680Sensor.h"
BME680Sensor bme680Sensor;
#else
NullSensor bme680Sensor;
#endif
#if __has_include(<Adafruit_DPS310.h>)
#include "Sensor/DPS310Sensor.h"
DPS310Sensor dps310Sensor;
#else
NullSensor dps310Sensor;
#endif
#if __has_include(<Adafruit_MCP9808.h>)
#include "Sensor/MCP9808Sensor.h"
MCP9808Sensor mcp9808Sensor;
#else
NullSensor mcp9808Sensor;
#endif
#if __has_include(<Adafruit_SHT31.h>)
#include "Sensor/SHT31Sensor.h"
SHT31Sensor sht31Sensor;
#else
NullSensor sht31Sensor;
#endif
#if __has_include(<Adafruit_LPS2X.h>)
#include "Sensor/LPS22HBSensor.h"
LPS22HBSensor lps22hbSensor;
#else
NullSensor lps22hbSensor;
#endif
#if __has_include(<Adafruit_SHTC3.h>)
#include "Sensor/SHTC3Sensor.h"
SHTC3Sensor shtc3Sensor;
#else
NullSensor shtc3Sensor;
#endif
#if __has_include(<Adafruit_VEML7700.h>)
#include "Sensor/VEML7700Sensor.h"
VEML7700Sensor veml7700Sensor;
#else
NullSensor veml7700Sensor;
#endif
#if __has_include(<Adafruit_TSL2591.h>)
#include "Sensor/TSL2591Sensor.h"
TSL2591Sensor tsl2591Sensor;
#else
NullSensor tsl2591Sensor;
#endif
#if __has_include(<ClosedCube_OPT3001.h>)
#include "Sensor/OPT3001Sensor.h"
OPT3001Sensor opt3001Sensor;
#else
NullSensor opt3001Sensor;
#endif
#if __has_include(<Adafruit_SHT4x.h>)
#include "Sensor/SHT4XSensor.h"
SHT4XSensor sht4xSensor;
#else
NullSensor sht4xSensor;
#endif
#if __has_include(<SparkFun_MLX90632_Arduino_Library.h>)
#include "Sensor/MLX90632Sensor.h"
MLX90632Sensor mlx90632Sensor;
#else
NullSensor mlx90632Sensor;
#endif
#if __has_include(<DFRobot_LarkWeatherStation.h>)
#include "Sensor/DFRobotLarkSensor.h"
DFRobotLarkSensor dfRobotLarkSensor;
#else
NullSensor dfRobotLarkSensor;
#endif
#if __has_include(<DFRobot_RainfallSensor.h>)
#include "Sensor/DFRobotGravitySensor.h"
DFRobotGravitySensor dfRobotGravitySensor;
#else
NullSensor dfRobotGravitySensor;
#endif
#if __has_include(<SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>)
#include "Sensor/NAU7802Sensor.h"
NAU7802Sensor nau7802Sensor;
#else
NullSensor nau7802Sensor;
#endif
#if __has_include(<Adafruit_BMP3XX.h>)
#include "Sensor/BMP3XXSensor.h"
BMP3XXSensor bmp3xxSensor;
#else
NullSensor bmp3xxSensor;
#endif
#if __has_include(<Adafruit_PCT2075.h>)
#include "Sensor/PCT2075Sensor.h"
PCT2075Sensor pct2075Sensor;
#else
NullSensor pct2075Sensor;
#endif

MeshtasticRAK12035Sensor soilSensor1(0x21);
MeshtasticRAK12035Sensor soilSensor2(0x22);
MeshtasticRAK12035Sensor soilSensor3(0x23);

RCWL9620Sensor rcwl9620Sensor;
CGRadSensSensor cgRadSens;
#endif
#ifdef T1000X_SENSOR_EN
#include "Sensor/T1000xSensor.h"
T1000xSensor t1000xSensor;
#endif
#ifdef SENSECAP_INDICATOR
#include "Sensor/IndicatorSensor.h"
IndicatorSensor indicatorSensor;
#endif
#define FAILED_STATE_SENSOR_READ_MULTIPLIER 10
#define DISPLAY_RECEIVEID_MEASUREMENTS_ON_SCREEN true

#include "graphics/ScreenFonts.h"
#include <Throttle.h>

EnvironmentTelemetryModule::EnvironmentTelemetryModule() :
    ProtobufModule("EnvironmentTelemetry", meshtastic_PortNum_TELEMETRY_APP, &meshtastic_Telemetry_msg),
    concurrency::OSThread("EnvironmentTelemetry"),
    firstTime(true),
    lastMeasurementPacket(nullptr),
    sendToPhoneIntervalMs(SECONDS_IN_MINUTE * 1000),
    lastSentToMesh(0),
    lastSentToPhone(0),
    sensor_read_error_count(0),
    sleepOnNextExecution(false)
{
    LOG_INFO("EnvironmentTelemetryModule: Constructor called.");
}

int32_t EnvironmentTelemetryModule::runOnce()
{
    if (sleepOnNextExecution) {
        sleepOnNextExecution = false;
        uint32_t nightyNightMs = Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.environment_update_interval,
                                                                   default_telemetry_broadcast_interval_secs);
        LOG_DEBUG("Sleep for %lums, then awake to send metrics again", nightyNightMs);
        doDeepSleep(nightyNightMs, true, false);
    }

    uint32_t result = UINT32_MAX;

    if (!(moduleConfig.telemetry.environment_measurement_enabled || moduleConfig.telemetry.environment_screen_enabled)) {
        return disable();
    }

    if (firstTime) {
        firstTime = 0;
        LOG_INFO("EnvTelModule: First run initialization.");

        if (moduleConfig.telemetry.environment_measurement_enabled) {
            LOG_INFO("Environment Telemetry: init sensors");

            LOG_INFO("RAK12035: Performing GLOBAL power cycle for RAK12023 module via WB_IO2 (pin %d)...", WB_IO2);
            pinMode(WB_IO2, OUTPUT);
            digitalWrite(WB_IO2, LOW);
            delay(100);
            digitalWrite(WB_IO2, HIGH);
            delay(500);
            
            LOG_INFO("RAK12035: Performing GLOBAL reset for RAK12023 module via WB_IO4 (pin %d)...", WB_IO4);
            pinMode(WB_IO4, OUTPUT);
            digitalWrite(WB_IO4, LOW);
            delay(500);
            digitalWrite(WB_IO4, HIGH);
            delay(500);

#ifdef SENSECAP_INDICATOR
            result = min(result, (uint32_t)indicatorSensor.runOnce());
#endif
#ifdef T1000X_SENSOR_EN
            result = min(result, (uint32_t)t1000xSensor.runOnce());
#elif !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL
            if (dfRobotLarkSensor.hasSensor()) result = min(result, (uint32_t)dfRobotLarkSensor.runOnce());
            if (dfRobotGravitySensor.hasSensor()) result = min(result, (uint32_t)dfRobotGravitySensor.runOnce());
            if (bmp085Sensor.hasSensor()) result = min(result, (uint32_t)bmp085Sensor.runOnce());
#if __has_include(<Adafruit_BME280.h>)
            if (bmp280Sensor.hasSensor()) result = min(result, (uint32_t)bmp280Sensor.runOnce());
#endif
            if (bme280Sensor.hasSensor()) result = min(result, (uint32_t)bme280Sensor.runOnce());
            if (ltr390uvSensor.hasSensor()) result = min(result, (uint32_t)ltr390uvSensor.runOnce());
            if (bmp3xxSensor.hasSensor()) result = min(result, (uint32_t)bmp3xxSensor.runOnce());
            if (bme680Sensor.hasSensor()) result = min(result, (uint32_t)bme680Sensor.runOnce());
            if (dps310Sensor.hasSensor()) result = min(result, (uint32_t)dps310Sensor.runOnce());
            if (mcp9808Sensor.hasSensor()) result = min(result, (uint32_t)mcp9808Sensor.runOnce());
            if (shtc3Sensor.hasSensor()) result = min(result, (uint32_t)shtc3Sensor.runOnce());
            if (lps22hbSensor.hasSensor()) result = min(result, (uint32_t)lps22hbSensor.runOnce());
            if (sht31Sensor.hasSensor()) result = min(result, (uint32_t)sht31Sensor.runOnce());
            if (sht4xSensor.hasSensor()) result = min(result, (uint32_t)sht4xSensor.runOnce());
            if (ina219Sensor.hasSensor()) result = min(result, (uint32_t)ina219Sensor.runOnce());
            if (ina260Sensor.hasSensor()) result = min(result, (uint32_t)ina260Sensor.runOnce());
            if (ina3221Sensor.hasSensor()) result = min(result, (uint32_t)ina3221Sensor.runOnce());
            if (veml7700Sensor.hasSensor()) result = min(result, (uint32_t)veml7700Sensor.runOnce());
            if (tsl2591Sensor.hasSensor()) result = min(result, (uint32_t)tsl2591Sensor.runOnce());
            if (opt3001Sensor.hasSensor()) result = min(result, (uint32_t)opt3001Sensor.runOnce());
            if (rcwl9620Sensor.hasSensor()) result = min(result, (uint32_t)rcwl9620Sensor.runOnce());
            if (aht10Sensor.hasSensor()) result = min(result, (uint32_t)aht10Sensor.runOnce());
            if (mlx90632Sensor.hasSensor()) result = min(result, (uint32_t)mlx90632Sensor.runOnce());
            if (nau7802Sensor.hasSensor()) result = min(result, (uint32_t)nau7802Sensor.runOnce());
            if (max17048Sensor.hasSensor()) result = min(result, (uint32_t)max17048Sensor.runOnce());
            if (cgRadSens.hasSensor()) result = min(result, (uint32_t)cgRadSens.runOnce());
            if (pct2075Sensor.hasSensor()) result = min(result, (uint32_t)pct2075Sensor.runOnce());

            soilSensor1.init();
            soilSensor2.init();
            soilSensor3.init();
            LOG_INFO("EnvTelModule: RAK12035 Sensor init results -> Soil1 detected=%d, Soil2 detected=%d, Soil3 detected=%d",
                     soilSensor1.hasSensor(), soilSensor2.hasSensor(), soilSensor3.hasSensor());

            if (soilSensor1.hasSensor() || soilSensor2.hasSensor() || soilSensor3.hasSensor()) {
                result = Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.environment_update_interval, default_telemetry_broadcast_interval_secs);
                LOG_INFO("EnvTelModule: At least one soil sensor detected, setting result to poll interval (%u ms).", result);
            }
#ifdef HAS_RAKPROT
            result = min(result, (uint32_t)rak9154Sensor.runOnce());
#endif
#endif
        }
        
        LOG_INFO("EnvTelModule: First run returning, final result=%u, disabling? %d", result, (result == UINT32_MAX));
        return result == UINT32_MAX ? disable() : setStartDelay();

    } else {
        if (!moduleConfig.telemetry.environment_measurement_enabled) {
            return disable();
        }
        
#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL
        if (bme680Sensor.hasSensor()) {
            result = min(result, (uint32_t)bme680Sensor.runTrigger());
        }
#endif

        if (((lastSentToMesh == 0) ||
             !Throttle::isWithinTimespanMs(lastSentToMesh, Default::getConfiguredOrDefaultMsScaled(
                                                                moduleConfig.telemetry.environment_update_interval,
                                                                default_telemetry_broadcast_interval_secs, numOnlineNodes))) &&
            airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
            airTime->isTxAllowedAirUtil()) {
            LOG_INFO("EnvTelModule: Attempting to send telemetry to mesh.");
            sendTelemetry();
            lastSentToMesh = millis();
        } else if (((lastSentToPhone == 0) || !Throttle::isWithinTimespanMs(lastSentToPhone, sendToPhoneIntervalMs)) &&
                   (service->isToPhoneQueueEmpty())) {
            LOG_INFO("EnvTelModule: Attempting to send telemetry to phone.");
            sendTelemetry(NODENUM_BROADCAST, true);
            lastSentToPhone = millis();
        }
    }
    
    return min((uint32_t)sendToPhoneIntervalMs, result);
}

bool EnvironmentTelemetryModule::wantUIFrame()
{
    return moduleConfig.telemetry.environment_screen_enabled;
}

void EnvironmentTelemetryModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);

    if (lastMeasurementPacket == nullptr) {
        display->drawString(x, y, "Environment");
        display->drawString(x, y += _fontHeight(FONT_SMALL), "No measurement");
        return;
    }

    meshtastic_Telemetry lastMeasurement;
    uint32_t agoSecs = service->GetTimeSinceMeshPacket(lastMeasurementPacket);
    const char *lastSender = getSenderShortName(*lastMeasurementPacket);

    const meshtastic_Data &p = lastMeasurementPacket->decoded;
    if (!pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &lastMeasurement)) {
        display->drawString(x, y, "Measurement Error");
        LOG_ERROR("Unable to decode last packet");
        return;
    }

    display->drawString(x, y, "Env. From: " + String(lastSender) + " (" + String(agoSecs) + "s)");

    String sensorData[10];
    int sensorCount = 0;

    if (lastMeasurement.variant.environment_metrics.has_temperature ||
        lastMeasurement.variant.environment_metrics.has_relative_humidity) {
        String last_temp = String(lastMeasurement.variant.environment_metrics.temperature, 0) + "°C";
        if (moduleConfig.telemetry.environment_display_fahrenheit) {
            last_temp =
                String(UnitConversions::CelsiusToFahrenheit(lastMeasurement.variant.environment_metrics.temperature), 0) + "°F";
        }
        sensorData[sensorCount++] =
            "Temp/Hum: " + last_temp + " / " + String(lastMeasurement.variant.environment_metrics.relative_humidity, 0) + "%";
    }

    if (lastMeasurement.variant.environment_metrics.barometric_pressure != 0) {
        sensorData[sensorCount++] =
            "Press: " + String(lastMeasurement.variant.environment_metrics.barometric_pressure, 0) + "hPA";
    }

    if (lastMeasurement.variant.environment_metrics.has_voltage) {
        sensorData[sensorCount++] = "Soil 1: " + String(lastMeasurement.variant.environment_metrics.voltage, 1) + "%";
    }
    if (lastMeasurement.variant.environment_metrics.has_current) {
        sensorData[sensorCount++] = "Soil 2: " + String(lastMeasurement.variant.environment_metrics.current, 1) + "%";
    }
    if (lastMeasurement.variant.environment_metrics.has_iaq) {
        sensorData[sensorCount++] = "Soil 3: " + String(lastMeasurement.variant.environment_metrics.iaq) + "%";
    }

    if (lastMeasurement.variant.environment_metrics.distance != 0) {
        sensorData[sensorCount++] = "Water Level: " + String(lastMeasurement.variant.environment_metrics.distance, 0) + "mm";
    }

    if (lastMeasurement.variant.environment_metrics.weight != 0) {
        sensorData[sensorCount++] = "Weight: " + String(lastMeasurement.variant.environment_metrics.weight, 0) + "kg";
    }

    if (lastMeasurement.variant.environment_metrics.radiation != 0) {
        sensorData[sensorCount++] = "Rad: " + String(lastMeasurement.variant.environment_metrics.radiation, 2) + "µR/h";
    }

    if (lastMeasurement.variant.environment_metrics.lux != 0) {
        sensorData[sensorCount++] = "Illuminance: " + String(lastMeasurement.variant.environment_metrics.lux, 2) + "lx";
    }

    if (lastMeasurement.variant.environment_metrics.white_lux != 0) {
        sensorData[sensorCount++] = "W_Lux: " + String(lastMeasurement.variant.environment_metrics.white_lux, 2) + "lx";
    }

    static int scrollOffset = 0;
    static bool scrollingDown = true;
    static uint32_t lastScrollTime = millis();

    static int maxLines = 0;
    if (!maxLines) {
        const int16_t paddingTop = _fontHeight(FONT_SMALL);
        const int16_t paddingBottom = 8;
        maxLines = (display->getHeight() - paddingTop - paddingBottom) / _fontHeight(FONT_SMALL);
        assert(maxLines > 0);
    }

    int linesToShow = min(maxLines, sensorCount);
    for (int i = 0; i < linesToShow; i++) {
        int index = (scrollOffset + i) % sensorCount;
        display->drawString(x, y += _fontHeight(FONT_SMALL), sensorData[index]);
    }

    if (sensorCount > maxLines) {
        if (millis() - lastScrollTime > 5000) {
            if (scrollingDown) {
                scrollOffset++;
                if (scrollOffset + linesToShow >= sensorCount) {
                    scrollingDown = false;
                }
            } else {
                scrollOffset--;
                if (scrollOffset <= 0) {
                    scrollingDown = true;
                }
            }
            lastScrollTime = millis();
        }
    }
}

bool EnvironmentTelemetryModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *t)
{
    if (t->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
#ifdef DEBUG_PORT
        const char *sender = getSenderShortName(mp);

        LOG_INFO("(Received from %s): barometric_pressure=%f, current=%f, gas_resistance=%f, relative_humidity=%f, "
                 "temperature=%f",
                 sender, t->variant.environment_metrics.barometric_pressure, t->variant.environment_metrics.current,
                 t->variant.environment_metrics.gas_resistance, t->variant.environment_metrics.relative_humidity,
                 t->variant.environment_metrics.temperature);
        LOG_INFO("(Received from %s): voltage=%f, IAQ=%u, distance=%f, lux=%f, white_lux=%f", sender,
                 t->variant.environment_metrics.voltage, t->variant.environment_metrics.iaq,
                 t->variant.environment_metrics.distance, t->variant.environment_metrics.lux,
                 t->variant.environment_metrics.white_lux);

        LOG_INFO("(Received from %s): wind speed=%fm/s, direction=%d degrees, weight=%fkg", sender,
                 t->variant.environment_metrics.wind_speed, t->variant.environment_metrics.wind_direction,
                 t->variant.environment_metrics.weight);

        LOG_INFO("Recv: radiation=%fµR/h", t->variant.environment_metrics.radiation);

#endif
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(mp);
    }

    return false;
}
// EnvironmentTelemetry.cpp (Final, Complete & Verified - Part 2/2)
bool EnvironmentTelemetryModule::getEnvironmentTelemetry(meshtastic_Telemetry *m)
{
    bool valid = true;
    bool hasSensor = false;
    m->time = getTime();
    m->which_variant = meshtastic_Telemetry_environment_metrics_tag;
    m->variant.environment_metrics = meshtastic_EnvironmentMetrics_init_zero;

#ifdef SENSECAP_INDICATOR
    valid = valid && indicatorSensor.getMetrics(m);
    hasSensor = true;
#endif
#ifdef T1000X_SENSOR_EN
    valid = valid && t1000xSensor.getMetrics(m);
    hasSensor = true;
#else
    if (dfRobotLarkSensor.hasSensor()) {
        valid = valid && dfRobotLarkSensor.getMetrics(m);
        hasSensor = true;
    }
    if (dfRobotGravitySensor.hasSensor()) {
        valid = valid && dfRobotGravitySensor.getMetrics(m);
        hasSensor = true;
    }
    if (sht31Sensor.hasSensor()) {
        valid = valid && sht31Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (sht4xSensor.hasSensor()) {
        valid = valid && sht4xSensor.getMetrics(m);
        hasSensor = true;
    }
    if (lps22hbSensor.hasSensor()) {
        valid = valid && lps22hbSensor.getMetrics(m);
        hasSensor = true;
    }
    if (shtc3Sensor.hasSensor()) {
        valid = valid && shtc3Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (bmp085Sensor.hasSensor()) {
        valid = valid && bmp085Sensor.getMetrics(m);
        hasSensor = true;
    }
#if __has_include(<Adafruit_BME280.h>)
    if (bmp280Sensor.hasSensor()) {
        valid = valid && bmp280Sensor.getMetrics(m);
        hasSensor = true;
    }
#endif
    if (bme280Sensor.hasSensor()) {
        valid = valid && bme280Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (ltr390uvSensor.hasSensor()) {
        valid = valid && ltr390uvSensor.getMetrics(m);
        hasSensor = true;
    }
    if (bmp3xxSensor.hasSensor()) {
        valid = valid && bmp3xxSensor.getMetrics(m);
        hasSensor = true;
    }
    if (bme680Sensor.hasSensor()) {
        valid = valid && bme680Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (dps310Sensor.hasSensor()) {
        valid = valid && dps310Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (mcp9808Sensor.hasSensor()) {
        valid = valid && mcp9808Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (ina219Sensor.hasSensor()) {
        valid = valid && ina219Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (ina260Sensor.hasSensor()) {
        valid = valid && ina260Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (ina3221Sensor.hasSensor()) {
        valid = valid && ina3221Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (veml7700Sensor.hasSensor()) {
        valid = valid && veml7700Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (tsl2591Sensor.hasSensor()) {
        valid = valid && tsl2591Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (opt3001Sensor.hasSensor()) {
        valid = valid && opt3001Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (mlx90632Sensor.hasSensor()) {
        valid = valid && mlx90632Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (rcwl9620Sensor.hasSensor()) {
        valid = valid && rcwl9620Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (nau7802Sensor.hasSensor()) {
        valid = valid && nau7802Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (aht10Sensor.hasSensor()) {
        if (!bmp280Sensor.hasSensor() && !bmp3xxSensor.hasSensor()) {
            valid = valid && aht10Sensor.getMetrics(m);
            hasSensor = true;
        } else if (bmp280Sensor.hasSensor()) {
            meshtastic_Telemetry m_ahtx = meshtastic_Telemetry_init_zero;
            LOG_INFO("AHTX0+BMP280 module detected: using temp from BMP280 and humy from AHTX0");
            aht10Sensor.getMetrics(&m_ahtx);
            m->variant.environment_metrics.relative_humidity = m_ahtx.variant.environment_metrics.relative_humidity;
            m->variant.environment_metrics.has_relative_humidity = m_ahtx.variant.environment_metrics.has_relative_humidity;
        } else {
            meshtastic_Telemetry m_ahtx = meshtastic_Telemetry_init_zero;
            LOG_INFO("AHTX0+BMP3XX module detected: using temp from BMP3XX and humy from AHTX0");
            aht10Sensor.getMetrics(&m_ahtx);
            m->variant.environment_metrics.relative_humidity = m_ahtx.variant.environment_metrics.relative_humidity;
            m->variant.environment_metrics.has_relative_humidity = m_ahtx.variant.environment_metrics.has_relative_humidity;
        }
    }
     // Temporary storage for soil temperatures
    float soilTemp1 = 0, soilTemp2 = 0, soilTemp3 = 0;
    bool hasTemp1 = false, hasTemp2 = false, hasTemp3 = false;

    // Recovery logic is now in RAK12035.cpp, this file is clean.
   // --- SENSOR DATA COLLECTION & MAPPING TO HUMIDITY GRAPH ---
// Read soil sensors and capture temperatures
    if (soilSensor1.hasSensor()) {
        meshtastic_Telemetry tempTelemetry = meshtastic_Telemetry_init_zero;
        if (soilSensor1.getMetrics(&tempTelemetry)) {
            soilTemp1 = tempTelemetry.variant.environment_metrics.temperature;
            hasTemp1 = true;
            // Store moisture in voltage field as before
            m->variant.environment_metrics.voltage = tempTelemetry.variant.environment_metrics.voltage;
            m->variant.environment_metrics.has_voltage = true;
        }
        hasSensor = true;
    }
    if (soilSensor2.hasSensor()) {
        meshtastic_Telemetry tempTelemetry = meshtastic_Telemetry_init_zero;
        if (soilSensor2.getMetrics(&tempTelemetry)) {
            soilTemp2 = tempTelemetry.variant.environment_metrics.temperature;
            hasTemp2 = true;
            // Store moisture in current field as before
            m->variant.environment_metrics.current = tempTelemetry.variant.environment_metrics.current;
            m->variant.environment_metrics.has_current = true;
        }
        hasSensor = true;
    }
    if (soilSensor3.hasSensor()) {
        meshtastic_Telemetry tempTelemetry = meshtastic_Telemetry_init_zero;
        if (soilSensor3.getMetrics(&tempTelemetry)) {
            soilTemp3 = tempTelemetry.variant.environment_metrics.temperature;
            hasTemp3 = true;
            // Store moisture in iaq field as before
            m->variant.environment_metrics.iaq = tempTelemetry.variant.environment_metrics.iaq;
            m->variant.environment_metrics.has_iaq = true;
        }
        hasSensor = true;
    }

    // Store soil temperatures in different fields
    if (hasTemp1) {
        m->variant.environment_metrics.temperature = soilTemp1;
        m->variant.environment_metrics.has_temperature = true;
    }
    if (hasTemp2) {
        m->variant.environment_metrics.barometric_pressure = soilTemp2;
        m->variant.environment_metrics.has_barometric_pressure = true;
    }
    if (hasTemp3) {
        m->variant.environment_metrics.gas_resistance = soilTemp3;
        m->variant.environment_metrics.has_gas_resistance = true;
    }
    
    if (max17048Sensor.hasSensor()) {
        valid = valid && max17048Sensor.getMetrics(m);
        hasSensor = true;
    }
    if (cgRadSens.hasSensor()) {
        valid = valid && cgRadSens.getMetrics(m);
        hasSensor = true;
    }
    if (pct2075Sensor.hasSensor()) {
        valid = valid && pct2075Sensor.getMetrics(m);
        hasSensor = true;
    }
#ifdef HAS_RAKPROT
    valid = valid && rak9154Sensor.getMetrics(m);
    hasSensor = true;
#endif
#endif
 LOG_INFO("EnvTel: valid=%d, hasSensor=%d. Soil1(V):%f, Soil2(C):%f, Soil3(IAQ):%u, Temp1:%f, Temp2:%f, Temp3:%f",
             valid, hasSensor,
             m->variant.environment_metrics.voltage,
             m->variant.environment_metrics.current,
             m->variant.environment_metrics.iaq,
             m->variant.environment_metrics.temperature,
             m->variant.environment_metrics.barometric_pressure,
             m->variant.environment_metrics.gas_resistance);

    return valid && hasSensor;
}

meshtastic_MeshPacket *EnvironmentTelemetryModule::allocReply()
{
    if (currentRequest) {
        auto req = *currentRequest;
        const auto &p = req.decoded;
        meshtastic_Telemetry scratch;
        meshtastic_Telemetry *decoded = NULL;
        memset(&scratch, 0, sizeof(scratch));
        if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &scratch)) {
            decoded = &scratch;
        } else {
            LOG_ERROR("Error decoding EnvironmentTelemetry module!");
            return NULL;
        }
        if (decoded->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
            meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
            if (getEnvironmentTelemetry(&m)) {
                LOG_INFO("Environment telemetry reply to request");
                return allocDataProtobuf(m);
            } else {
                return NULL;
            }
        }
    }
    return NULL;
}

bool EnvironmentTelemetryModule::sendTelemetry(NodeNum dest, bool phoneOnly)
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.which_variant = meshtastic_Telemetry_environment_metrics_tag;
    m.time = getTime();

    if (getEnvironmentTelemetry(&m)) {
        LOG_INFO("Send: barometric_pressure=%f, current=%f, gas_resistance=%f, relative_humidity=%f, temperature=%f",
                 m.variant.environment_metrics.barometric_pressure, m.variant.environment_metrics.current,
                 m.variant.environment_metrics.gas_resistance, m.variant.environment_metrics.relative_humidity,
                 m.variant.environment_metrics.temperature);
        LOG_INFO("Send: voltage=%f, IAQ=%u, distance=%f, lux=%f", m.variant.environment_metrics.voltage,
                 m.variant.environment_metrics.iaq, m.variant.environment_metrics.distance, m.variant.environment_metrics.lux);
        LOG_INFO("Send: wind speed=%fm/s, direction=%d degrees, weight=%fkg", m.variant.environment_metrics.wind_speed,
                 m.variant.environment_metrics.wind_direction, m.variant.environment_metrics.weight);
        LOG_INFO("Send: radiation=%fµR/h", m.variant.environment_metrics.radiation); 

        sensor_read_error_count = 0;

        meshtastic_MeshPacket *p = allocDataProtobuf(m);
        p->to = dest;
        p->decoded.want_response = false;
        if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
            p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
        else
            p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
            
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(*p);
        if (phoneOnly) {
            LOG_INFO("Send packet to phone");
            service->sendToPhone(p);
        } else {
            LOG_INFO("Send packet to mesh");
            service->sendToMesh(p, RX_SRC_LOCAL, true);

            if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR && config.power.is_power_saving) {
                meshtastic_ClientNotification *notification = clientNotificationPool.allocZeroed();
                notification->level = meshtastic_LogRecord_Level_INFO;
                notification->time = getValidTime(RTCQualityFromNet);
                sprintf(notification->message, "Sending telemetry and sleeping for %lu interval in a moment",
                        Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.environment_update_interval,
                                                          default_telemetry_broadcast_interval_secs) /
                            1000U);
                service->sendClientNotification(notification);
                sleepOnNextExecution = true;
                LOG_DEBUG("Start next execution in 5s, then sleep");
                setIntervalFromNow(FIVE_SECONDS_MS);
            }
        }
        return true;
    }
    return false;
}

AdminMessageHandleResult EnvironmentTelemetryModule::handleAdminMessageForModule(const meshtastic_MeshPacket &mp,
                                                                                 meshtastic_AdminMessage *request,
                                                                                 meshtastic_AdminMessage *response)
{
    AdminMessageHandleResult result = AdminMessageHandleResult::NOT_HANDLED;
#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL
    if (dfRobotLarkSensor.hasSensor()) {
        result = dfRobotLarkSensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (dfRobotGravitySensor.hasSensor()) {
        result = dfRobotGravitySensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (sht31Sensor.hasSensor()) {
        result = sht31Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (lps22hbSensor.hasSensor()) {
        result = lps22hbSensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (shtc3Sensor.hasSensor()) {
        result = shtc3Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bmp085Sensor.hasSensor()) {
        result = bmp085Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bmp280Sensor.hasSensor()) {
        result = bmp280Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bme280Sensor.hasSensor()) {
        result = bme280Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (ltr390uvSensor.hasSensor()) {
        result = ltr390uvSensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bmp3xxSensor.hasSensor()) {
        result = bmp3xxSensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (bme680Sensor.hasSensor()) {
        result = bme680Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (dps310Sensor.hasSensor()) {
        result = dps310Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (mcp9808Sensor.hasSensor()) {
        result = mcp9808Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (ina219Sensor.hasSensor()) {
        result = ina219Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (ina260Sensor.hasSensor()) {
        result = ina260Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (ina3221Sensor.hasSensor()) {
        result = ina3221Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (veml7700Sensor.hasSensor()) {
        result = veml7700Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (tsl2591Sensor.hasSensor()) {
        result = tsl2591Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (opt3001Sensor.hasSensor()) {
        result = opt3001Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (mlx90632Sensor.hasSensor()) {
        result = mlx90632Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (rcwl9620Sensor.hasSensor()) {
        result = rcwl9620Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (nau7802Sensor.hasSensor()) {
        result = nau7802Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (aht10Sensor.hasSensor()) {
        result = aht10Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (max17048Sensor.hasSensor()) {
        result = max17048Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (soilSensor1.hasSensor()) {
        result = soilSensor1.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (soilSensor2.hasSensor()) {
        result = soilSensor2.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (soilSensor3.hasSensor()) {
        result = soilSensor3.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (cgRadSens.hasSensor()) {
        result = cgRadSens.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
    if (pct2075Sensor.hasSensor()) {
        result = pct2075Sensor.handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }
#ifdef HAS_RAKPROT
    result = rak9154Sensor.handleAdminMessage(mp, request, response);
    if (result != AdminMessageHandleResult::NOT_HANDLED)
        return result;
#endif
#endif
    return result;
}
#endif