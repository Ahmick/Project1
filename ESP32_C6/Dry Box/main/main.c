#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme280.h"
#include "driver/i2c_master.h"

static const char *TAG = "BME280";
static i2c_bus_handle_t i2c_bus = NULL;
static bme280_handle_t bme280 = NULL;

#define I2C_MASTER_SCL_IO           4                           /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           5                           /*!< GPIO number used for I2C master data  */
//#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000



void app_main(void)
{
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
        float temperature = 0.0, humidity = 0.0, pressure = 0.0;
        bme280_read_temperature(bme280, &temperature);
        bme280_read_humidity(bme280, &humidity);
        bme280_read_pressure(bme280, &pressure);
        if (temperature!=0.0){
            ESP_LOGI(TAG, "Temp: %.2f, Hum: %.2f, Pres: %.0f", temperature, humidity, pressure);
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
