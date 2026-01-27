#include <stdio.h>
#include <string.h> 
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme280.h"
#include "driver/i2c_master.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_SSID   "COGECO-ABD4F8"
#define WIFI_PASS   "kdmmnwn2"



#define I2C_MASTER_SCL_IO           4                           /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           5                           /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

TickType_t start_tick = 0;
TaskHandle_t taskHandle = NULL;
static const char *TAG = "MQTT_SUB";
static const char *TAG1 = "wifi station";
static const char *TAG2 = "DELAY_TASK";
static const char *TAG3 = "BME280";
static i2c_bus_handle_t i2c_bus = NULL;
static bme280_handle_t bme280 = NULL;
int i = 0;
int k = 0;
int run_time = 0;


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
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        //int msg_id_1 = esp_mqtt_client_subscribe(client, "1", 0);
        //int msg_id_2 = esp_mqtt_client_subscribe(client, "2", 0);
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
        printf("Published\n");
        break;
    
    //Prints event data. If data  "1 or 2" sets i to 1 or 2. 
    
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len, event->data);
        // char buf[32];
        // if (strncmp(event->topic, "1", event->topic_len) == 0)
        // {
        //     memcpy(buf, event->data, event->data_len);
        //     buf[event->data_len] = '\0';
        //     k = atoi(buf);
        //     i = 1; //Relay 1 turn on 
        //     xTaskNotifyGive(taskHandle);

        // }
        // if (strncmp(event->topic, "2", event->topic_len) == 0)
        // {
        //     memcpy(buf, event->data, event->data_len);
        //     buf[event->data_len] = '\0';
        //     k = atoi(buf);
        //     i = 2; //Relay 2 identifier
        //     xTaskNotifyGive(taskHandle);

        // }
        
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        printf("Shits fucked\n");
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

void build_message(char *out_buf, size_t out_sz, float a, float b, float c)
{
    printf("build_message");
    snprintf(out_buf, out_sz, "%02.1f,%02.1f,%04.0f", a, b, c);
}


void app_main(void)
{
    char message[16];
    float temperature = 0.0, humidity = 0.0, pressure = 0.0;

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
    
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    bme280 = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT);
    ESP_ERROR_CHECK(bme280_default_init(bme280));
    ESP_ERROR_CHECK(bme280_set_sampling(bme280, 
                    BME280_MODE_FORCED, 
                    BME280_SAMPLING_X1, 
                    BME280_SAMPLING_X1, 
                    BME280_SAMPLING_X1, 
                    BME280_FILTER_OFF, 
                    BME280_STANDBY_MS_1000));
        
        
    while (1){
        ESP_ERROR_CHECK(bme280_take_forced_measurement(bme280));
        bme280_read_temperature(bme280, &temperature);
        bme280_read_humidity(bme280, &humidity);
        bme280_read_pressure(bme280, &pressure);
        if (temperature!=0.0){
            ESP_LOGI(TAG3, "Temp: %.2f, Hum: %.2f, Pres: %.0f", temperature, humidity, pressure);

            build_message(message, 16, temperature, humidity, pressure);
            esp_mqtt_client_publish(client, "Dry Box", message, strlen(message),2,0);
        }

        

        vTaskDelay(pdMS_TO_TICKS(30000));
        }
}
