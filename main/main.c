/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


//     Includes         //
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "bme280.h"


//      DEFINES       //
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_25

#define ACK_CHECK_EN 0x1                                       /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                                      /*!< I2C master will not check ack from slave */

#define ACK_VAL 0x0                                            /*!< I2C ack value */
#define NACK_VAL 0x1                                           /*!< I2C nack value */

#define READ_BIT  1                           
#define WRITE_BIT 0
#define I2C_MASTER_NUM 0                                      /*!< I2C port number for master dev */

#define I2C_DEVICE_ADDRESS BME280_I2C_ADDR_PRIM               // options = BME280_I2C_ADDR_PRIM or BME280_I2C_ADDR_SEC



//      FUNCTIONS       //

esp_err_t i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  //  printf("\n read function is called\n");
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    if (len == 0) 
        {
            return ESP_OK;
        }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( I2C_DEVICE_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_DEVICE_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);

    if (len > 1)
        {
            i2c_master_read(cmd, reg_data, len - 1, ACK_VAL);
        }
        i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
        i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
    
    if( ret != ESP_OK )
        {
            printf("\nfailed in read function\n");
             rslt = 1;
        }
        i2c_cmd_link_delete(cmd);
        return rslt;
}
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
   // printf("\n write function is called\n"); 
    int8_t rslt = 0;        // return 0 for sucess non 0 for failure


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_DEVICE_ADDRESS << 1) | WRITE_BIT, true);
   
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if( ret == ESP_OK )
    {
        return rslt;
    }
    else
    {
        printf("failed in write function");
        return 1;
    }
}

void user_delay_ms(uint32_t period, void *intf_ptr)
{
    vTaskDelay( period / portTICK_PERIOD_MS);
}
void user_delay_us(uint32_t period, void *intf_ptr)   
{
    ets_delay_us(period);
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}
void print_chip_info()
{
     printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
/*
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");

*/
    fflush(stdout);
   // esp_restart();
};

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

	printf("Temperature, Pressure, Humidity\r\n");
	while (1) {
		/* Delay while the sensor completes a measurement */
		dev->delay_us(70000, dev->intf_ptr);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
		print_sensor_data(&comp_data);
	}

	return rslt;
}
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
	uint32_t req_delay;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);
	
	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&dev->settings);

    printf("Temperature, Pressure, Humidity\r\n");
    /* Continuously stream sensor data */
    while (1) {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_us(req_delay, dev->intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        print_sensor_data(&comp_data);
    }
    return rslt;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    print_chip_info();

    struct bme280_dev dev;
    int8_t rslt = BME280_OK;
    uint8_t dev_addr = I2C_DEVICE_ADDRESS;

    dev.intf_ptr = &dev_addr;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
    rslt = bme280_init(&dev);

    rslt = stream_sensor_data_normal_mode(&dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
        exit(1);
    }
}
