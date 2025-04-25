#include "ZigbeeMeterSensor.h"
#if CONFIG_ZB_ENABLED

// esp_zb_cluster_list_t *zigbee_occupancy_sensor_clusters_create(zigbee_occupancy_sensor_cfg_t *occupancy_sensor) {

esp_zb_cluster_list_t *esp_zb_electrical_meas_clusters_create(zigbee_meas_sensor_cfg_t *occupancy_sensor) {
  esp_zb_basic_cluster_cfg_t *basic_cfg = occupancy_sensor ? &(occupancy_sensor->basic_cfg) : NULL;
  esp_zb_identify_cluster_cfg_t *identify_cfg = occupancy_sensor ? &(occupancy_sensor->identify_cfg) : NULL;
  esp_zb_electrical_meas_cluster_cfg_t *elec_meas_cfg = occupancy_sensor ? &(occupancy_sensor->elec_meas_cfg) : NULL;
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster_create(basic_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  // esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, esp_zb_occupancy_sensing_cluster_create(electrical_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_electrical_meas_cluster(cluster_list, esp_zb_electrical_meas_cluster_create(elec_meas_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  return cluster_list;
}

//esp_zb_electrical_meas_cluster_create

ZigbeeMeterSensor::ZigbeeMeterSensor(uint8_t endpoint) : ZigbeeEP(endpoint) {
  _device_id = ESP_ZB_HA_METER_INTERFACE_DEVICE_ID;
  _humidity_sensor = false;

  //esp_zb_temperature_sensor_cfg_t temp_sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
  //_cluster_list = esp_zb_temperature_sensor_clusters_create(&temp_sensor_cfg);

  //esp_zb_on_off_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();

  zigbee_meas_sensor_cfg_t electrical_cfg = ESP_ZB_DEFAULT_METER_SENSOR_CONFIG();
  //_cluster_list = esp_zb_electrical_meas_clusters_create(&electrical_cfg);
  _ep_config = {
    .endpoint = _endpoint, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_METER_INTERFACE_DEVICE_ID, .app_device_version = 0
  };
}

static int16_t zb_float_to_s16(float temp) {
  return (int16_t)(temp * 100);
}

bool ZigbeeMeterSensor::setMinMaxValue(float min, float max) {
  int16_t zb_min = zb_float_to_s16(min);
  int16_t zb_max = zb_float_to_s16(max);
  esp_zb_attribute_list_t *temp_measure_cluster =
    esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_err_t ret = esp_zb_cluster_update_attr(temp_measure_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, (void *)&zb_min);
  if (ret != ESP_OK) {
    log_e("Failed to set min value: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  ret = esp_zb_cluster_update_attr(temp_measure_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, (void *)&zb_max);
  if (ret != ESP_OK) {
    log_e("Failed to set max value: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  return true;
}

bool ZigbeeMeterSensor::setReporting(uint16_t min_interval, uint16_t max_interval, float delta) {
  esp_zb_zcl_reporting_info_t reporting_info;
  memset(&reporting_info, 0, sizeof(esp_zb_zcl_reporting_info_t));
  reporting_info.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
  reporting_info.ep = _endpoint;
  reporting_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT;
  reporting_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
  //reporting_info.attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
  reporting_info.attr_id = ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID;
  reporting_info.u.send_info.min_interval = min_interval;
  reporting_info.u.send_info.max_interval = max_interval;
  reporting_info.u.send_info.def_min_interval = min_interval;
  reporting_info.u.send_info.def_max_interval = max_interval;
  reporting_info.u.send_info.delta.u16 = (uint16_t)(delta * 100);  // Convert delta to ZCL uint16_t
  reporting_info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
  reporting_info.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_err_t ret = esp_zb_zcl_update_reporting_info(&reporting_info);
  esp_zb_lock_release();
  if (ret != ESP_OK) {
    log_e("Failed to set reporting: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  return true;
}

bool ZigbeeMeterSensor::setTemperature(float temperature) {
  esp_zb_zcl_status_t ret = ESP_ZB_ZCL_STATUS_SUCCESS;
  int16_t zb_temperature = zb_float_to_s16(temperature);
  log_v("Updating temperature sensor value...");
  /* Update temperature sensor measured value */
  log_d("Setting temperature to %d", zb_temperature);
  esp_zb_lock_acquire(portMAX_DELAY);
  ret = esp_zb_zcl_set_attribute_val(
    _endpoint, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &zb_temperature, false
  );
  esp_zb_lock_release();
  if (ret != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Failed to set temperature: 0x%x: %s", ret, esp_zb_zcl_status_to_name(ret));
    return false;
  }
  return true;
}

bool ZigbeeMeterSensor::reportTemperature() {
  /* Send report attributes command */
  esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
  report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID;
  report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
  report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT;
  report_attr_cmd.zcl_basic_cmd.src_endpoint = _endpoint;
  report_attr_cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
  esp_zb_lock_release();
  if (ret != ESP_OK) {
    log_e("Failed to send temperature report: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  log_v("Temperature report sent");
  return true;
}

/*
void ZigbeeMeterSensor::addHumiditySensor(float min, float max, float tolerance) {
  int16_t zb_min = zb_float_to_s16(min);
  int16_t zb_max = zb_float_to_s16(max);
  uint16_t zb_tolerance = (uint16_t)(tolerance * 100);
  int16_t default_hum = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT;
  esp_zb_attribute_list_t *humidity_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
  esp_zb_humidity_meas_cluster_add_attr(humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &default_hum);
  esp_zb_humidity_meas_cluster_add_attr(humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &zb_min);
  esp_zb_humidity_meas_cluster_add_attr(humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &zb_max);
  esp_zb_humidity_meas_cluster_add_attr(humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_TOLERANCE_ID, &zb_tolerance);
  esp_zb_cluster_list_add_humidity_meas_cluster(_cluster_list, humidity_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  _humidity_sensor = true;
}
*/

/*
bool ZigbeeMeterSensor::setHumidity(float humidity) {
  esp_zb_zcl_status_t ret = ESP_ZB_ZCL_STATUS_SUCCESS;
  int16_t zb_humidity = zb_float_to_s16(humidity);
  log_v("Updating humidity sensor value...");
  log_d("Setting humidity to %d", zb_humidity);
  esp_zb_lock_acquire(portMAX_DELAY);
  ret = esp_zb_zcl_set_attribute_val(
    _endpoint, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &zb_humidity,
    false
  );
  esp_zb_lock_release();
  if (ret != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Failed to set humidity: 0x%x: %s", ret, esp_zb_zcl_status_to_name(ret));
    return false;
  }
  return true;
}
*/

/*
bool ZigbeeMeterSensor::reportHumidity() {
  esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
  report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
  report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
  report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
  report_attr_cmd.zcl_basic_cmd.src_endpoint = _endpoint;
  report_attr_cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
  esp_zb_lock_release();
  if (ret != ESP_OK) {
    log_e("Failed to send humidity report: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  log_v("Humidity report sent");
  return true;
}
*/
/* bool ZigbeeMeterSensor::setHumidityReporting(uint16_t min_interval, uint16_t max_interval, float delta) {
  esp_zb_zcl_reporting_info_t reporting_info;
  memset(&reporting_info, 0, sizeof(esp_zb_zcl_reporting_info_t));
  reporting_info.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
  reporting_info.ep = _endpoint;
  reporting_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
  reporting_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
  reporting_info.attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
  reporting_info.u.send_info.min_interval = min_interval;
  reporting_info.u.send_info.max_interval = max_interval;
  reporting_info.u.send_info.def_min_interval = min_interval;
  reporting_info.u.send_info.def_max_interval = max_interval;
  reporting_info.u.send_info.delta.u16 = (uint16_t)(delta * 100);  // Convert delta to ZCL uint16_t
  reporting_info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
  reporting_info.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_err_t ret = esp_zb_zcl_update_reporting_info(&reporting_info);
  esp_zb_lock_release();
  if (ret != ESP_OK) {
    log_e("Failed to set humidity reporting: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  return true;
}
*/

bool ZigbeeMeterSensor::report() {
  bool temp_ret = reportTemperature();
  bool hum_ret = true;
  if (_humidity_sensor) {
    hum_ret = reportHumidity();
  }
  return temp_ret && hum_ret;
}

#endif  // CONFIG_ZB_ENABLED
