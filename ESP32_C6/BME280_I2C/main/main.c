/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a BME280 temp, humidity and pressure sensor.
*/
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
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BME280_SENSOR_ADDR         0x76        /*!< Address of the MPU9250 sensor */
#define BME280_WHO_AM_I_REG_ADDR   0xD0        /*!< Register addresses of the "who am I" register */
#define BME280_STS_REG_ADDR        0xE0        /*!< Register addresses of the status register. Bit 0: 1 when copying, 0 when done. Bit 3: 1 when measuring, 0 when done*/
#define BME280_RESET_BIT           0xB6

// /**
//  * @brief Read a sequence of bytes from a BME280 sensor registers
//  */
// static esp_err_t bme280_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// /**
//  * @brief Write a byte to a MPU9250 sensor register
//  */
// static esp_err_t bme280_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
// {
//     uint8_t write_buf[2] = {reg_addr, data};
//     return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// /**
//  * @brief i2c master initialization
//  */
// static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
// {
//     i2c_master_bus_config_t bus_config = {
//         .i2c_port = I2C_MASTER_NUM,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };
//     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

//     i2c_device_config_t dev_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = BME280_SENSOR_ADDR,
//         .scl_speed_hz = I2C_MASTER_FREQ_HZ,
//     };
//     ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
// }

void app_main(void)
{
    //uint8_t data[2];
    //i2c_master_bus_handle_t bus_handle;
    //i2c_master_dev_handle_t dev_handle;
    //i2c_master_init(&bus_handle, &dev_handle);
    //ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the BME280 WHO_AM_I register, on power up the register should have the value 0x60 */
    //ESP_ERROR_CHECK(bme280_register_read(dev_handle, BME280_WHO_AM_I_REG_ADDR, data, 1));
    //ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    
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
    

    /* Demonstrate writing by resetting the MPU9250 */
    //ESP_ERROR_CHECK(bme280_register_write_byte(dev_handle, BME280_STS_REG_ADDR, BME280_RESET_BIT));


    /* Read the BME280 WHO_AM_I register, on power up the register should have the value 0x60 */
    //ESP_ERROR_CHECK(bme280_register_read(dev_handle, BME280_WHO_AM_I_REG_ADDR, data, 1));
    //ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    //ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    //ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    //ESP_LOGI(TAG, "I2C de-initialized successfully");
}
