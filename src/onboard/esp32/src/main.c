#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

#include "network.h"

static const char *TAG = "OTA";

// extern const uint8_t cert_pem_start[] asm("_binary_cert_pem_start");
// extern const uint8_t cert_pem_end[]   asm("_binary_cert_pem_end");

void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    strncpy((char *)wifi_config.sta.ssid, get_ssid(), sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, get_password(), sizeof(wifi_config.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Connecting to WiFi...");
    esp_wifi_connect();
}

void ota_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting OTA...");

    const char *hostname = get_hostname();
    char url_str[128] = "https://";
    strncat(url_str, hostname, sizeof(url_str) - strlen(url_str) - 1);
    strncat(url_str, ":8080/firmware.bin", sizeof(url_str) - strlen(url_str) - 1);

    esp_http_client_config_t http_config = {
        .url = url_str,
        .cert_pem = NULL, //(const char *)cert_pem_start,
        .skip_cert_common_name_check = true,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .timeout_ms = 10000,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful, restarting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(ret));
    }

    vTaskDelete(NULL);
}

void ota_task_forever(void *pvParameter) {
    while (1) {
        printf("Starting OTA task...\n");
        ota_task(NULL);
        vTaskDelay(pdMS_TO_TICKS(10000));  // Retry every 60 seconds
    }
}



void app_main(void) {
    nvs_flash_init();
    wifi_init();

    vTaskDelay(pdMS_TO_TICKS(5000));  // wait for WiFi to connect
    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
}