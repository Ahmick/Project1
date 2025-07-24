#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_SSID   "COGECO-9E02B8"
#define WIFI_PASS   "ktyut2m2"
#define GPIO_1 2
#define GPIO_2 4

int i = 0;
int k = 0;
int z = 1;
static const char *TAG = "MQTT_SUB";
static const char *TAG1 = "wifi station";
static const char *TAG2 = "DELAY_TASK";

//setup GPIO 1-2
void setup_gpio_inputs() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_1) |
                        (1ULL << GPIO_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
}



// Define the task function
void task(void *pvParameters)
{
    TickType_t start_tick = 0; // xTaskGetTickCount(); //Gets the current tick and keeps it. Used to get elapsed time
    TickType_t current_tick = 0;
    //i = 0;
    // Gets the current tick, prints the elapsed time and delays 1 second.

    while (true){
        current_tick = xTaskGetTickCount();
        z = (current_tick - start_tick) * portTICK_PERIOD_MS 

        if (i == 1 && z > 0) {
            gpio_set_level(GPIO_1, 1);
            ESP_LOGI(TAG2, "GPIO1 high");
            start_tick = current_tick;
            current_tick = xTaskGetTickCount();
            //vTaskDelay(1000 / portTICK_PERIOD_MS);
            
        } else if (i == 2 && z > 0) {
            gpio_set_level(GPIO_2, 1);
            ESP_LOGI(TAG2, "GPIO2 high");
            start_tick = current_tick; 
            current_tick = xTaskGetTickCount();
            //vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
            
            gpio_set_level(GPIO_1, 0);
            gpio_set_level(GPIO_2, 0);
            current_tick = xTaskGetTickCount();
            start_tick = current_tick; 
            ESP_LOGI(TAG2, "GPIO1 and 2 low");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            z = 1;
        }
        }
        
        current_tick = xTaskGetTickCount(); 
        uint32_t elapsed_ms = (current_tick - start_tick) * portTICK_PERIOD_MS;
        ESP_LOGI(TAG2, "Elapsed Time: %d ms", elapsed_ms);
        if (elapsed_ms > i*60000 && k == 1){
            i = 0;
            ESP_LOGI(TAG2, "Timer reset to 0 and i set to 0");
            start_tick = 0; // xTaskGetTickCount(); //Gets the current tick and keeps it. Used to get elapsed time
            k = 0;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 1 second

    }

    
}

//Sets up MQTT server

esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://homeassistant.local",
        .credentials.username = "mosquitto",
        .credentials.authentication.password= "Mosquitto",
    };

//Set up the handler. 
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        int msg_id = esp_mqtt_client_subscribe(client, "1", 0);
        int msg_id = esp_mqtt_client_subscribe(client, "2", 0);
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        printf("Published/n");
        break;
    
    //Prints event data. If data  "1 or 2" sets i to 1 or 2. 
    
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        
        if (strncmp(event->topic, "1", event->topic_len) == 0)
        {
            i = 1; //Relay 1 turn on 
            //k = event->data; // Message sent tag
        }
        if (strncmp(event->topic, "2", event->topic_len) == 0)
        {
            i = 2; //Relay 2 identifier
            //k = event->data; //Message sent tag
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        printf("Shits fucked");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


//Everything below is to setup WiFi


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1



static int s_retry_num = 0;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG1, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG1,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG1, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
         * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    printf("Connected...\n");
    
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    printf("Debug1\n");
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    printf("Debug2\n");
    printf("%d\n",i);
    esp_mqtt_client_start(client);
    printf("Debug3\n");

    setup_gpio_inputs();
    // Create the FreeRTOS task
    xTaskCreate(
        task,              // Task function
        "task",            // Task name
        2048,              // Stack size in bytes
        NULL,              // Parameters to pass
        5,                 // Task priority
        NULL               // Task handle
    );
    printf("Debug4\n");
    printf("%d\n",i);
}
