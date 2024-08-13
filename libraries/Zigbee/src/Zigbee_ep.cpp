/* Common Class for Zigbee End Point */

#include "Zigbee_ep.h"
#include "esp_zigbee_cluster.h"

uint8_t Zigbee_EP::_endpoint = 0;
bool Zigbee_EP::_is_bound = false;
bool Zigbee_EP::_allow_multiple_binding = false;

/* Zigbee End Device Class */
Zigbee_EP::Zigbee_EP(uint8_t endpoint) {
    _endpoint = endpoint;
    _ep_config.endpoint = 0;
    _cluster_list = nullptr;
    _attribute_cluster = nullptr;
}

Zigbee_EP::~Zigbee_EP() {

}

void Zigbee_EP::setManufacturerAndModel(const char *name, const char *model) {
    // Convert manufacturer to ZCL string
    size_t length = strlen(name);
    if (length > 32) {
        log_e("Manufacturer name is too long");
        return;
    }
    // Allocate a new array of size length + 2 (1 for the length, 1 for null terminator)
    char* zb_name = new char[length + 2];
    // Store the length as the first element
    zb_name[0] = static_cast<char>(length); // Cast size_t to char
    // Use memcpy to copy the characters to the result array
    memcpy(zb_name + 1, name, length);
    // Null-terminate the array
    zb_name[length + 1] = '\0';

    // Convert model to ZCL string
    length = strlen(model);
    if (length > 32) {
        log_e("Model name is too long");
        delete[] zb_name;
        return;
    }
    char* zb_model = new char[length + 2];
    zb_model[0] = static_cast<char>(length);
    memcpy(zb_model + 1, model, length);
    zb_model[length + 1] = '\0';

    // Get the basic cluster and update the manufacturer and model attributes
    esp_zb_attribute_list_t *basic_cluster = esp_zb_cluster_list_get_cluster(_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)zb_name);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)zb_model);

    //NOTE:
    // esp_zb_cluster_add_attr function
}

void Zigbee_EP::readManufacturerAndModel(uint8_t endpoint, uint16_t short_addr) {
    /* Read peer Manufacture Name & Model Identifier */
    esp_zb_zcl_read_attr_cmd_t read_req;
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    read_req.zcl_basic_cmd.src_endpoint = _endpoint;
    read_req.zcl_basic_cmd.dst_endpoint = endpoint;
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC;

    uint16_t attributes[] = {
    ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
    ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
    };
    read_req.attr_number = ZB_ARRAY_LENTH(attributes);
    read_req.attr_field = attributes;

    esp_zb_zcl_read_attr_cmd_req(&read_req);
}

void Zigbee_EP::attribute_read_cmd(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute) {
    /* Basic cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID && attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING && attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char *)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            log_i("Peer Manufacturer is \"%s\"", string);
            readManufacturer(string);
            free(string);
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID && attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING && attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char *)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            log_i("Peer Model is \"%s\"", string);
            readModel(string);
            free(string);
        }
    }
}