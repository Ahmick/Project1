#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define WIFI_SSID      "COGECO-9E02B8"
#define WIFI_PASSWORD  "ktyut2m2"


void initialize(void)
{
    printf("Initializing memory, network and creating event loop. \n");
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    printf("Initializing done.");
}

void app_main(void)
{
    initialize();
    
    esp_wifi_start();
    esp_wifi_connect();
    printf("Wifi connected \n");
    printf("That is it for now. \n");
}
