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
#include "i2c_eeprom.h"

#define SCL_IO_PIN 15
#define SDA_IO_PIN 23
#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY
#define PORT_NUMBER -1
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
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_eeprom_config_t eeprom_config = {
        .eeprom_device.scl_speed_hz = MASTER_FREQUENCY,
        .eeprom_device.device_address = 0x77,
        .addr_wordlen = 2,
        .write_time_ms = 10,
    };

    i2c_eeprom_handle_t eeprom_handle;


    ESP_ERROR_CHECK(i2c_eeprom_init(bus_handle, &eeprom_config, &eeprom_handle));

    for (int j = 0; j < 1, j++;)
    {
        //ESP_ERROR_CHECK(i2c_eeprom_write(eeprom_handle, block_addr, buf, LENGTH));
        // Needs wait for eeprom hardware done, referring from datasheet
        block_addr = 0x73;
        i2c_eeprom_wait_idle(eeprom_handle);
        ESP_ERROR_CHECK(i2c_eeprom_read(eeprom_handle, block_addr, read_buf, LENGTH));
        disp_buf(read_buf, LENGTH);
        vTaskDelay(50);
    }
}
