#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"

#define GAS_SENSOR_PIN GPIO_NUM_21 // GPIO pin for MQ6 gas sensor
#define DEVICE_NAME "ESP32_GasSensor" // BLE device name

#define TAG "GAS_SENSOR_BLE"

// BLE UUIDs
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890AB"
#define CHAR_UUID "12345678-1234-1234-1234-1234567890AC"

// BLE variables
static uint16_t service_handle = 0;
static uint16_t char_handle = 0;
static esp_gatt_char_prop_t char_property = 0;

// Gas sensor variables
static uint8_t gas_status = 0; // 0 = No gas detected, 1 = Gas detected

// Task to monitor gas sensor
void gas_detection_task(void *param) {
    gpio_set_direction(GAS_SENSOR_PIN, 1);

    while (1) {
        uint8_t current_status = gpio_get_level(GAS_SENSOR_PIN);
        ESP_LOGI(TAG, "Gas sensor GPIO level: %d", gpio_get_level(GAS_SENSOR_PIN));

        if (current_status != gas_status) {
            gas_status = current_status;

            // Log gas detection status
             ESP_LOGI(TAG, "Sensor State: %d, Char Handle: %d, Service Handle: %d",
                 gas_status, char_handle, service_handle);

            if (gas_status == 1) {
                ESP_LOGI(TAG, "Gas detected!");
            } else {
                ESP_LOGI(TAG, "No gas detected.");
            }

            // Send BLE notification
            if (char_handle != 0 && service_handle != 0) {
                esp_err_t err = esp_ble_gatts_send_indicate(0, service_handle, char_handle, 
                    sizeof(gas_status), &gas_status, false);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Notification sent: %d", gas_status);
                } else {
                    ESP_LOGE(TAG, "Failed to send notification: %s", esp_err_to_name(err));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check gas status every second

    }
}

// BLE GATT event handler
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "Registering service...");
            esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t) {
                .is_primary = true,
                .id.inst_id = 0,
                .id.uuid.len = ESP_UUID_LEN_128,
                .id.uuid.uuid.uuid128 = {0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x56, 0x78, 0x90, 0xAB}
            }, 4);
            break;

        case ESP_GATTS_CREATE_EVT:
            service_handle = param->create.service_handle;
            ESP_LOGI(TAG, "Service created, adding characteristic...");
            esp_ble_gatts_add_char(service_handle, &(esp_bt_uuid_t) {
                .len = ESP_UUID_LEN_128,
                .uuid.uuid128 = {0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x56, 0x78, 0x90, 0xAC}
            }, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 
               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, 
               NULL, NULL);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Characteristic added, starting service...");
            esp_ble_gatts_start_service(service_handle);
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started, starting BLE advertising...");
            esp_ble_gap_start_advertising(&(esp_ble_adv_params_t) {
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
            });
            break;

        default:
            ESP_LOGW(TAG, "Unhandled GATT event: %d", event);
            break;
    }
}

// BLE GAP event handler
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE advertising started.");
            } else {
                ESP_LOGE(TAG, "BLE advertising failed.");
            }
            break;

        default:
            ESP_LOGW(TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

// Main application entry point
void app_main() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));

    // Start gas detection task
    xTaskCreate(gas_detection_task, "gas_detection_task", 4096, NULL, 5, NULL);
}
