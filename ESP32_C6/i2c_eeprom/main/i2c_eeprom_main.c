/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "i2c_eeprom.h"

#define SCL_IO_PIN 6
#define SDA_IO_PIN 5
#define MASTER_FREQUENCY 1000000
#define PORT_NUMBER 0
#define LENGTH 48

static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

void app_main(void)
{
    vTaskDelay(30);
    esp_err_t err;
    int j = 0;
    int i = 0;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    for (i=0; i < 127; i++){
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = i,
            .scl_speed_hz = 100000,
            };
        
            printf("%02x ", i);
        
        if (j == 7 ){
            printf("\n");
            j = 0;
        }else{
            j++;
        }
        

        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
        err = i2c_master_probe(bus_handle, i, -1);

        if (err != ESP_ERR_NOT_FOUND){
            ESP_ERROR_CHECK(i2c_master_probe(bus_handle, i, -1));
            printf("\n \n Address %02x has device... \n", i);
            //i = 127;
        }
        ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
        vTaskDelay(pdMS_TO_TICKS(20));
        }
}