/* Class of Zigbee Temperature + Humidity sensor endpoint inherited from common EP class */

#pragma once

#include "soc/soc_caps.h"
#include "sdkconfig.h"
#if CONFIG_ZB_ENABLED

#include "ZigbeeEP.h"
#include "ha/esp_zigbee_ha_standard.h"

// clang-format off
#define ESP_ZB_DEFAULT_METER_SENSOR_CONFIG()                                                  \
  {                                                                                               \
      .basic_cfg =                                                                                \
          {                                                                                       \
              .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                          \
              .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,                        \
          },                                                                                      \
      .identify_cfg =                                                                             \
          {                                                                                       \
              .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,                   \
          },                                                                                      \
      .elec_meas_cfg =                                                                            \
          {                                                                                       \
              .measured_value = ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID,               \
              .min_value = ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MIN_ID,                \
              .max_value = ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MAX_ID,                \
          },                                                                                      \
  }
// clang-format on

typedef struct zigbee_meas_sensor_cfg_s {
  esp_zb_basic_cluster_cfg_t basic_cfg;
  esp_zb_identify_cluster_cfg_t identify_cfg;
  esp_zb_electrical_meas_cluster_cfg_t elec_meas_cfg;
} zigbee_meas_sensor_cfg_t;


class ZigbeeMeterSensor : public ZigbeeEP {
public:
  ZigbeeMeterSensor(uint8_t endpoint);
  ~ZigbeeMeterSensor() {}

  // Set the temperature value in 0,01°C
  bool setTemperature(float value);

  // Set the min and max value for the temperature sensor in 0,01°C
  bool setMinMaxValue(float min, float max);

  // Set the tolerance value for the temperature sensor in 0,01°C
  bool setTolerance(float tolerance);

  // Set the reporting interval for temperature measurement in seconds and delta (temp change in 0,01 °C)
  bool setReporting(uint16_t min_interval, uint16_t max_interval, float delta);

  // Report the temperature value
  bool reportTemperature();

  // Add humidity cluster to the temperature sensor device
  void addHumiditySensor(float min, float max, float tolerance);

  // Set the humidity value in 0,01%
  bool setHumidity(float value);

  // Set the reporting interval for humidity measurement in seconds and delta (humidity change in 0,01%)
  bool setHumidityReporting(uint16_t min_interval, uint16_t max_interval, float delta);

  // Report the humidity value
  bool reportHumidity();

  // Report the temperature and humidity values if humidity sensor is added
  bool report();

private:
  bool _humidity_sensor;
};

#endif  // CONFIG_ZB_ENABLED
