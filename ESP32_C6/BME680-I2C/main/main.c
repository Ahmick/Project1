#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "BME680";

#define I2C_MASTER_SCL_IO           15         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           23         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SENSOR_ADDRESS 0x77        /*!< Address of the BME680 sensor */
#define ID_REG_ADDR    0xD0        /*!< Register addresses of the "ID" register */


//Config Registers
#define ctrl_hum       0x72   /*!< Register address of the Oversampling setting of humidity register osrs_t<7:5> and osrs_h<2:0> */            
#define ctrl_meas      0x74   /*!< Register address of the Oversampling setting of temp, and press register osrs_t<7:5> and osrs_h<2:0> */     
#define config         0x75   /*!< Register address of the IIR filter register filter<4:2> */                                  

//Gas heater DAC set point
#define idac_heat_0    0X50   /*!< Register address of the gas hear DAC register <7:0> */
#define idac_heat_1    0X51   /*!< Register address of the gas hear DAC register <7:0> */
#define idac_heat_2    0X52   /*!< Register address of the gas hear DAC register <7:0> */
#define idac_heat_3    0X53   /*!< Register address of the gas hear DAC register <7:0> */
#define idac_heat_4    0X54   /*!< Register address of the gas hear DAC register <7:0> */
#define idac_heat_5    0X55   /*!< Register address of the gas hear DAC register <7:0> */
#define idac_heat_6    0X56   /*!< Register address of the gas hear DACregister <7:0> */
#define idac_heat_7    0X57   /*!< Register address of the gas hear DACregister <7:0> */
#define idac_heat_8    0X58   /*!< Register address of the gas hear DACregister <7:0> */
#define idac_heat_9    0X59   /*!< Register address of the gas hear DACregister <7:0> */

//Gas heater resistance set points
#define res_wait_0    0X5A   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_1    0X5B   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_2    0X5C   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_3    0X5D   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_4    0X5E   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_5    0X5F   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_6    0X60   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_7    0X61   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_8    0X62   /*!< Register address of the target heater resistancec register <7:0> */
#define res_wait_9    0X63   /*!< Register address of the target heater resistancec register <7:0> */


//Data Registers
//Pressure Data
#define PRESS_MSB      0x1F   /*!< Register address of pressure msb register */                    //content bit press_msb<7:0>
#define PRESS_LSB      0x20   /*!< Register address of pressure lsb register */                    //content bit press_lsb<7:0>
#define PRESS_XLSB     0x21   /*!< Register address of pressure xlsb register */                   //content bit press_xlsb<7:4>

//Temp Data
#define TEMP_MSB       0x22   /*!< Register address of temperature msb register */                 //content bit temp_msb<7:0>
#define TEMP_LSB       0x23   /*!< Register address of temperature lsb register */                 //content bit temp_lsb<7:0>
#define TEMP_XLSB      0x24   /*!< Register address of temperature xlsb register */                //content bit temp_xlsb<7:4>

//Humidity Data
#define HUM_MSB        0x25   /*!< Register address of humidity msb register */                    //content bit temp_msb<7:0>
#define HUM_LSB        0x26   /*!< Register address of humidity lsb register */                    //content bit temp_lsb<7:0>

//Gas Sensor Resistance Data
#define GAS_R_MSB      0x2A   /*!< Register address of gas resistance msb register */              //content bit temp_msb<7:0>
#define GAS_R_LSB      0x2B   /*!< Register address of gas resistance and range lsb register */    //content bit temp_lsb<7:6>, gas_range_r<3:0> 


//Status Registers
#define meas_staus     0x1D   /*!< Register address of status flag register */                     //content bit new_data<7:0>


/**
 * @brief Read a sequence of bytes from a BME680 sensor registers
 */
static esp_err_t bme680_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a BME680 sensor register
 */
static esp_err_t bme680_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SENSOR_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static double temperature_calc(i2c_master_dev_handle_t dev_handle)
{
    uint32_t temp_adc, temp_temp_adc, par_t1, par_t2;
    uint8_t par_t11, par_t12, par_t21, par_t22, par_t3, temp_adc1, temp_adc2, temp_adc3, data[2];
    double var1, var2, var3, t_fine, temp_comp;

    //Reads necessary regsters.
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0xE9, data, 1));    
    par_t11 = data[0];
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0xEA, data, 1));    
    par_t12 = data[0];
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0x8A, data, 1));    
    par_t21 = data[0];
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0x8B, data, 1));    
    par_t22 = data[0];
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0x8C, data, 1));    
    par_t3 = data[0];
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0x22, data, 1));    
    temp_adc1 = data[0];
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0x23, data, 1));    
    temp_adc2 = data[0];
    ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0x24, data, 1));    
    temp_adc3 = data[0];


    //Debugging logs
    ESP_LOGI(TAG, "part11 = %X", par_t11);
    ESP_LOGI(TAG, "part12 = %X", par_t12);
    ESP_LOGI(TAG, "part21 = %X", par_t21);
    ESP_LOGI(TAG, "part22 = %X", par_t22);
    ESP_LOGI(TAG, "part3 = %X", par_t3);
    ESP_LOGI(TAG, "adc1 = %X", temp_adc1);
    ESP_LOGI(TAG, "adc2 = %X", temp_adc2);
    ESP_LOGI(TAG, "adc3 = %X", temp_adc3);

    //Takes the 8 bit memory and turns to 16 bit data (20 bit for temp_adc)
    par_t1 = (par_t12 << 8) | par_t11;    
    par_t2 = (par_t22 << 8) | par_t21;   
    temp_temp_adc = (temp_adc1 << 8) | temp_adc2;    
    temp_adc3 = (temp_adc3 >> 4) &0x0F;    
    temp_adc = temp_temp_adc << 4 | temp_adc3;    
    
    //Debuggins logs
    ESP_LOGI(TAG, "par_t1 = %lX", par_t1);
    ESP_LOGI(TAG, "par_t2 = %lX", par_t2);
    ESP_LOGI(TAG, "16 bit temp_adc = %lX", temp_adc);
    ESP_LOGI(TAG, "4 bit temp_adc3 = %X", temp_adc3);
    ESP_LOGI(TAG, "20 bit temp_adc = %lX", temp_adc);

    printf("par_t1= %lx, par_t2= %lx, par_t3= %x, temp_adc= %lx \n", par_t1, par_t2, par_t3, temp_adc);
    
    /* This is an alternative calculation given by datasheet.
    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1 * (int32_t)par_t2) >> 11;
    var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14;
    t_fine = var2 + var3;
    temp_comp = ((t_fine * 5) + 128) >> 8;
    */
    
    var1 = (((double)temp_adc / 16384.0) - ((double)par_t1 / 1024.0)) * (double)par_t2;
    var2 = ((((double)temp_adc / 131072.0) - ((double)par_t1 / 8192.0)) * (((double)temp_adc / 131072.0) - ((double)par_t1 / 8192.0))) * ((double)par_t3 * 16.0);
    t_fine = var1 + var2;
    temp_comp = t_fine / 5120.0;

    return temp_comp;
}

static void measurement(i2c_master_dev_handle_t dev_handle)
{
    int i = 1;
    uint8_t status;
    uint8_t data[2];
    
    ESP_ERROR_CHECK(bme680_register_write_byte(dev_handle, ctrl_hum, 0x3 )); //oversampling hum 4x
    ESP_ERROR_CHECK(bme680_register_write_byte(dev_handle, ctrl_meas, 0x6C )); //oversampling 4x, sleep mode 
    ESP_ERROR_CHECK(bme680_register_write_byte(dev_handle, 0x71, 0x0 )); //gas measurment off
    ESP_ERROR_CHECK(bme680_register_write_byte(dev_handle, config, 0x09 )); //temp filter 3x
    ESP_ERROR_CHECK(bme680_register_write_byte(dev_handle, ctrl_meas, 0x6D )); //measurment mode
    

    while (i == 1)
    {
        ESP_ERROR_CHECK(bme680_register_read(dev_handle, 0x1D , data, 1));
        status = data[0];
        i = (status >> 5) & 0x01;
        //ESP_LOGI(TAG, "data[0] = %X, data[1] = %X", data[0],data[1]);
        //ESP_LOGI(TAG, "status = %X, i = %d", status,i);
        vTaskDelay(pdMS_TO_TICKS(100));
        //ESP_ERROR_CHECK(bme680_register_read(dev_handle, , data, 1));
        // = 0;
    }
}

void app_main(void)
{
    double temp_comp;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");


    while (1)
    {
        measurement(dev_handle); //Sets the BME680 to take a measurment. Stops when measurement is done.

        temp_comp = temperature_calc(dev_handle); //Reads needed memory and calculates the temperature in degrees C.

        printf("Temp: %f \n", temp_comp);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    
}