#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"

// BLE Configuration
#define DEVICE_NAME "GasSensorBLE"
#define GAS_SENSOR_PIN GPIO_NUM_21 // MQ9 sensor pin

static const char *TAG = "BLE_GAS_SENSOR";
static uint8_t adv_service_uuid128[16] = {
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x12, 0x34,
    0x12, 0x34, 0x12, 0x34, 0x90, 0xAB, 0x00, 0x00
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

bool gas_detected = false;

void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE advertising started successfully.");
            } else {
                ESP_LOGE(TAG, "Failed to start BLE advertising, error code: %d", param->adv_start_cmpl.status);
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE advertising stopped successfully.");
            } else {
                ESP_LOGE(TAG, "Failed to stop BLE advertising, error code: %d", param->adv_stop_cmpl.status);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

void ble_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Bluetooth controller
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Initialize Bluedroid stack
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register GAP callback
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_gap_event_handler));

    // Configure advertising data
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
}

// Task to monitor the gas sensor
void gas_sensor_task(void *param) {
    gpio_set_direction(GAS_SENSOR_PIN, GPIO_MODE_INPUT);
    
    while (1) {
        int sensor_state = gpio_get_level(GAS_SENSOR_PIN); // Read sensor pin
        ESP_LOGI(TAG, "Sensor state: %d", sensor_state);

        if (sensor_state == 1 && !gas_detected) {
            gas_detected = true;
            ESP_LOGI(TAG, "Gas detected! Sending BLE notification...");
            // BLE notification logic here (if required, add a custom characteristic)
        } else if (sensor_state == 0 && gas_detected) {
            gas_detected = false;
            ESP_LOGI(TAG, "Gas cleared.");
        }

        // Delay for stability
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing BLE...");
    ble_init();

    ESP_LOGI(TAG, "Starting BLE advertising...");
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));

    ESP_LOGI(TAG, "Monitoring gas sensor...");
    xTaskCreate(gas_sensor_task, "gas_sensor_task", 2048, NULL, 5, NULL);
}
