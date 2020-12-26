/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

//     Includes         //
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "tftspi.h"
#include "tft.h"
#include "spiffs_vfs.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_heap_caps.h"

#ifdef CONFIG_EXAMPLE_USE_WIFI
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include <sys/time.h>
#include <unistd.h>
#include "lwip/err.h"
#include "apps/sntp/sntp.h"
#include "esp_log.h"
#include "nvs_flash.h"
#endif

#include "bme280.h"

#include "driver/rmt.h"
#include "led_strip.h"
#define LED_RMT_TX_CHANNEL RMT_CHANNEL_0
#define LED_CHASE_SPEED_MS (20)

#include "driver/adc.h"
#include "esp_adc_cal.h"
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static const char *TAG = "BME280 & WS2812B";
#define STORAGE_NAMESPACE "storage"

//      DEFINES BME280/ i2c      //
#define SDA_PIN GPIO_NUM_27			//14
#define SCL_PIN GPIO_NUM_26			//26
#define ACK_CHECK_EN 0x1                                       /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                                      /* I2C master will not check ack from slave */
#define ACK_VAL 0x0                                            /* I2C ack value */
#define NACK_VAL 0x1                                           /* I2C nack value */
#define READ_BIT  1                           
#define WRITE_BIT 0
#define I2C_MASTER_NUM 0                                      /* I2C port number for master dev */
#define I2C_DEVICE_ADDRESS BME280_I2C_ADDR_PRIM               // options = BME280_I2C_ADDR_PRIM or BME280_I2C_ADDR_SEC

//------------------ADC------------------------

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6;     //ADC_CHANNEL_6 = GPIO34		GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

uint32_t LastAdc;
uint32_t getAdc()
{
	 //Check if Two Point or Vref are burned into eFuse
//  check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) 
	{
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    }
	 else 
	{
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
//  print_char_val_type(val_type);

    //Continuously sample ADC1
   // while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) 
		{
            if (unit == ADC_UNIT_1) 
			{
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } 
			else 
			{
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
  //    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
  //    printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
  //    vTaskDelay(pdMS_TO_TICKS(1000));
		free(adc_chars);
		return adc_reading;
   // }
}

//---------------------ADC_END---------------------------------


// ==========================================================
// Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST
#define SPI_BUS TFT_HSPI_HOST
// ==========================================================

static uint8_t doprint = 1;			//controls if printf statments used
static struct tm *tm_info;
static char tmp_buff[64], tmp_buff_last[64];
static time_t time_now, time_last = 0;

//==================================================================================
#ifdef CONFIG_EXAMPLE_USE_WIFI

static const char tag[] = "[TFT Demo]";

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = 0x00000001;

//------------------------------------------------------------
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		/* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

//-------------------------------
static void initialise_wifi(void)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_WIFI_SSID,
			.password = CONFIG_WIFI_PASSWORD,
		},
	};
	ESP_LOGI(tag, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

//-------------------------------
static void initialize_sntp(void)
{
	ESP_LOGI(tag, "Initializing SNTP");
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, "pool.ntp.org");
	sntp_init();
}

//--------------------------
static int obtain_time(void)
{
	int res = 1;
	initialise_wifi();
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

	initialize_sntp();

	// wait for time to be set
	int retry = 0;
	const int retry_count = 20;

	time(&time_now);
	tm_info = localtime(&time_now);

	while (tm_info->tm_year < (2016 - 1900) && ++retry < retry_count)
	{
		//ESP_LOGI(tag, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
		sprintf(tmp_buff, "Wait %0d/%d", retry, retry_count);
		TFT_print(tmp_buff, CENTER, LASTY);
		vTaskDelay(500 / portTICK_RATE_MS);
		time(&time_now);
		tm_info = localtime(&time_now);
	}
	if (tm_info->tm_year < (2016 - 1900))
	{
		ESP_LOGI(tag, "System time NOT set.");
		res = 0;
	}
	else
	{
		ESP_LOGI(tag, "System time is set.");
	}

	ESP_ERROR_CHECK(esp_wifi_stop());
	return res;
}

#endif //CONFIG_EXAMPLE_USE_WIFI

//==================================================================================
//-----------------------------------START OF FUNCTIONS---------------------------

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}
led_strip_t *strip;		//made strip global to access it in print function

//----------------------
static void _checkTime()
{
	time(&time_now);
	if (time_now > time_last)
	{
		color_t last_fg, last_bg;
		time_last = time_now;
		tm_info = localtime(&time_now);
		sprintf(tmp_buff, "%02d:%02d:%02d", tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);

		TFT_saveClipWin();
		TFT_resetclipwin();

		Font curr_font = cfont;
		last_bg = _bg;
		last_fg = _fg;
		_fg = TFT_YELLOW;
		_bg = (color_t){64, 64, 64};
		TFT_setFont(DEFAULT_FONT, NULL);

		TFT_fillRect(1, _height - TFT_getfontheight() - 8, _width - 3, TFT_getfontheight() + 6, _bg);
		TFT_print(tmp_buff, CENTER, _height - TFT_getfontheight() - 5);

		cfont = curr_font;
		_fg = last_fg;
		_bg = last_bg;

		TFT_restoreClipWin();
	}
}
/*
//----------------------
static int _checkTouch()
{
	int tx, ty;
	if (TFT_read_touch(&tx, &ty, 0)) {
		while (TFT_read_touch(&tx, &ty, 1)) {
			vTaskDelay(20 / portTICK_RATE_MS);
		}
		return 1;
	}
	return 0;
}
*/
//---------------------
static int Wait(int ms)
{
	uint8_t tm = 1;
	if (ms < 0)
	{
		tm = 0;
		ms *= -1;
	}
	if (ms <= 50)
	{
		vTaskDelay(ms / portTICK_RATE_MS);
		//if (_checkTouch()) return 0;
	}
	else
	{
		for (int n = 0; n < ms; n += 50)
		{
			vTaskDelay(50 / portTICK_RATE_MS);
			if (tm)
				_checkTime();
			//if (_checkTouch()) return 0;
		}
	}
	return 1;
}
//-------------------------------------------------------------------
static unsigned int rand_interval(unsigned int min, unsigned int max)
{
	int r;
	const unsigned int range = 1 + max - min;
	const unsigned int buckets = RAND_MAX / range;
	const unsigned int limit = buckets * range;

	/* Create equal size buckets all in a row, then fire randomly towards
     * the buckets until you land in one of them. All buckets are equally
     * likely. If you land off the end of the line of buckets, try again. */
	do
	{
		r = rand();
	} while (r >= limit);

	return min + (r / buckets);
}
// Generate random color
//-----------------------------
static color_t random_color()
{

	color_t color;
	color.r = (uint8_t)rand_interval(8, 252);
	color.g = (uint8_t)rand_interval(8, 252);
	color.b = (uint8_t)rand_interval(8, 252);
	return color;
}
//---------------------
static void _dispTime()
{
	Font curr_font = cfont;
	if (_width < 240)
		TFT_setFont(DEF_SMALL_FONT, NULL);
	else
		TFT_setFont(DEFAULT_FONT, NULL);

	time(&time_now);
	time_last = time_now;
	tm_info = localtime(&time_now);
	sprintf(tmp_buff, "%02d:%02d:%02d", tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
	TFT_print(tmp_buff, CENTER, _height - TFT_getfontheight() - 5);

	cfont = curr_font;
}
//---------------------------------
static void disp_header(char *info)
{
	TFT_fillScreen(TFT_BLACK);
	TFT_resetclipwin();

	_fg = TFT_YELLOW;
	_bg = (color_t){64, 64, 64};

	if (_width < 240)
		TFT_setFont(DEF_SMALL_FONT, NULL);
	else
		TFT_setFont(DEFAULT_FONT, NULL);
	TFT_fillRect(0, 0, _width - 1, TFT_getfontheight() + 8, _bg);
	TFT_drawRect(0, 0, _width - 1, TFT_getfontheight() + 8, TFT_CYAN);

	TFT_fillRect(0, _height - TFT_getfontheight() - 9, _width - 1, TFT_getfontheight() + 8, _bg);
	TFT_drawRect(0, _height - TFT_getfontheight() - 9, _width - 1, TFT_getfontheight() + 8, TFT_CYAN);

	TFT_print(info, CENTER, 4);
	_dispTime();

	_bg = TFT_BLACK;
	TFT_setclipwin(0, TFT_getfontheight() + 9, _width - 1, _height - TFT_getfontheight() - 10);
}
//---------------------------------------------
static void update_header(char *hdr, char *ftr)
{
	color_t last_fg, last_bg;

	TFT_saveClipWin();
	TFT_resetclipwin();

	Font curr_font = cfont;
	last_bg = _bg;
	last_fg = _fg;
	_fg = TFT_YELLOW;
	_bg = (color_t){64, 64, 64};
	if (_width < 240)
		TFT_setFont(DEF_SMALL_FONT, NULL);
	else
		TFT_setFont(DEFAULT_FONT, NULL);

	if (hdr)
	{
		TFT_fillRect(1, 1, _width - 3, TFT_getfontheight() + 6, _bg);
		TFT_print(hdr, CENTER, 4);
	}

	if (ftr)
	{
		TFT_fillRect(1, _height - TFT_getfontheight() - 8, _width - 3, TFT_getfontheight() + 6, _bg);
		if (strlen(ftr) == 0)
			_dispTime();
		else
			TFT_print(ftr, CENTER, _height - TFT_getfontheight() - 5);
	}

	cfont = curr_font;
	_fg = last_fg;
	_bg = last_bg;

	TFT_restoreClipWin();
}

//==============================================================================
// Formula	:	RH = 100% X (E/Es)		where E is vapour pressure & Es is saturation vapour pressure
//				E =  EO * EXP( (L/Rv) * ((1/TO)-(1/Td)) )				
//				Es = EO * EXP( (L/Rv) * ((1/TO)-(1/T)) )
// Given  	:	EO = 0.611KPa; (L/Rv) = 5423 kelvin; TO = 273 Kelvin; (1/TO) = 0.003663;
// Therefore:	
const int 		dew_EO = 611;		// in Pa
double 			dew_Es = 0;
const uint16_t 	LRv = 5423;
void 	dew_from_humidity(struct bme280_data *comp_data)
{
	printf("\nTemp: %f", comp_data->temperature );
	float T = (1 / comp_data->temperature);
	printf("\n1/Temp: %f", T );

	T = 0.003663 - T;
	printf("\n(1/TO)-(1/T): %f", T );

	T *= LRv;
	printf("\n (1/TO)-(1/T) * (L/Rv): %f", T );
	
/*
	double TT;
	TT = exp(abs(T));
	printf("\nEXP((1/TO)-(1/T) * (L/Rv)): %f", TT );
*/
	double TT = T;
	TT = expl(abs(TT));
	printf("\nEXP((1/TO)-(1/T) * (L/Rv)): %f", TT );

	dew_Es = dew_EO * TT;
	printf("\ndew_Es: %f", dew_Es );

//	printf("\nsize of char:  %d", sizeof(char));
//	printf("\nsize of int:  %d", sizeof(int));
//	printf("\nsize of double:  %d", sizeof(double));
//	printf("\nsize of float:  %d", sizeof(float));
//	printf("\nsize of long double:  %d", sizeof(long double));
//	dew_Es = dew_EO * (exp((0.003663 - (1 / comp_data->temperature)) * 5423 ));
//	printf( "\nDew_Es ie Saturation vapour pressure:  %f", dew_Es );
}
//==============================================================================

//     BME280 & I2C FUNCTIONS       //

void delay_ms(uint32_t period, void *intf_ptr)
{
    vTaskDelay( period / portTICK_PERIOD_MS);
}
void delay_us(uint32_t period, void *intf_ptr)   
{
    ets_delay_us(period);
}

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
int8_t    bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
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
            printf("\nfailed in read function");
			printf("\nError (%s) reading from sensor\n",esp_err_to_name(ret));
            rslt = 1;
        }
        i2c_cmd_link_delete(cmd);
        return rslt;
}
int8_t    bme280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
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
int8_t 	  bme280_setup(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t dev_addr = I2C_DEVICE_ADDRESS;
    dev->intf_ptr = &dev_addr;
    dev->intf = BME280_I2C_INTF;
    dev->read = bme280_i2c_read;
    dev->write = bme280_i2c_write;
    dev->delay_us = delay_us;
    rslt = bme280_init( dev );
    return rslt;
}

int32_t GmaxTemp = 0;
int32_t GminTemp = 0x7FFFFFFF;					//MaxValue 
esp_err_t save_limits_temperature(void); 		//forward dec
void   	  print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE

	//printf("Free Heap size: %d \n",xPortGetFreeHeapSize());

		update_header(NULL, "");		// increments count in info footer
       // printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
		
		int32_t temp = comp_data->temperature;
		if( temp  > GmaxTemp )
		{
			GmaxTemp = comp_data->temperature;
		}
		if( temp < GminTemp )
		{
			GminTemp = comp_data->temperature;
		}
		save_limits_temperature();
	
		TFT_setFont(DEJAVU18_FONT, NULL);
		int x = 10;
		int y =  TFT_getfontheight() + 4;
		_fg = TFT_WHITE;
	//	TFT_setFont(DEJAVU18_FONT, NULL);
		sprintf(tmp_buff,"Temperature: %0.2f",comp_data->temperature);
		TFT_print(tmp_buff, x, y);
		y += TFT_getfontheight() + 4;
	
	//	TFT_setFont(DEJAVU18_FONT, NULL);
		sprintf(tmp_buff,"Max/MinTemp: %d / %d", GmaxTemp, GminTemp);
		TFT_print(tmp_buff, x, y);
		y += TFT_getfontheight() + 4;

	//	TFT_setFont(DEJAVU18_FONT, NULL);
		sprintf(tmp_buff,"Pressure: %0.2f",comp_data->pressure);
		TFT_print(tmp_buff, x, y);
		y += (TFT_getfontheight() + 4);

	//	TFT_setFont(DEJAVU18_FONT, NULL);
		sprintf(tmp_buff,"Humidity: %0.2f",comp_data->humidity);
		TFT_print(tmp_buff, x, y);
		y += (TFT_getfontheight() + 4);

		uint32_t AdcResult = getAdc();
	
	//	TFT_setFont(DEJAVU18_FONT, NULL);
		sprintf(tmp_buff,"Bar Graph: %d", AdcResult );
	
		if( strlen(tmp_buff) < strlen(tmp_buff_last) )
		{
			_fg = TFT_BLACK;
			TFT_print(tmp_buff_last, x, y);
			_fg = TFT_WHITE;
		}
		TFT_print(tmp_buff, x, y);
		sprintf(tmp_buff_last, tmp_buff);
	
		y += (TFT_getfontheight() + 4);				// this section calculates the color based on adc result
		x = 10;
		color_t myColor = {255, 255, 255};
		myColor.g = (uint32_t)((float)AdcResult/4095 * 255);
		myColor.r = 255 - myColor.g;
		myColor.b = 0;

		TFT_drawRect(x, y, 100, 20, TFT_YELLOW);				// draw empty box to fill based on adc result
		AdcResult = (uint32_t)((float)AdcResult/4095 * 99);
		if( AdcResult < LastAdc)
		{
			TFT_fillRect( x + AdcResult,  y+1, LastAdc-AdcResult, 18, TFT_BLACK);		//this clears the part of the rectangle drawn last frame if the new one is smaler 
		}
		TFT_fillRect( x,  y+1,  AdcResult, 18, myColor);
		LastAdc = AdcResult;

		for (int j = 0; j < CONFIG_STRIP_LED_NUMBER; j += 1)
		{
			ESP_ERROR_CHECK(strip->set_pixel(strip, j, myColor.r, myColor.g, myColor.b));
		}
		// Flush RGB values to LEDs
		ESP_ERROR_CHECK(strip->refresh(strip, 100)); 
		//vTaskDelay(pdMS_TO_TICKS(LED_CHASE_SPEED_MS));
		//strip->clear(strip, 50);
		//vTaskDelay(pdMS_TO_TICKS(LED_CHASE_SPEED_MS));
	

#else
        printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}
void 	  stream_sensor_data_normal_mode(void *ignore)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	struct bme280_dev dev;                                  //Structure to hold bme interface pointers. used in bme280_setup(&dev), USED TO OPPERATE THE BME280 CHIP
    ESP_ERROR_CHECK(i2c_master_init());                     // initiate I2C buss
    rslt = bme280_setup(&dev);                       		// set up bme280 interface
    if (rslt != BME280_OK)
        {
            printf( "Failed to set up bme280 (code %+d).", rslt);
    
        }

	/* Recommended mode of operation: Indoor navigation */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

	printf("Temperature, Pressure, Humidity\r\n");
	while (1) {
		/* Delay while the sensor completes a measurement */
		dev.delay_us(70000, dev.intf_ptr);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if (rslt != BME280_OK)
        {
            printf( "Failed to get sensor data (code %+d).", rslt);
            break;
        }
		print_sensor_data(&comp_data);
	}
	vTaskDelete(NULL);
	//return rslt;
}
void 	  stream_sensor_data_forced_mode(void *ignore)
{
    int8_t rslt;
    uint8_t settings_sel;
	uint32_t req_delay;
    struct bme280_data comp_data;

	struct bme280_dev dev;                                  //Structure to hold bme interface pointers. used in bme280_setup(&dev), USED TO OPPERATE THE BME280 CHIP
    ESP_ERROR_CHECK(i2c_master_init());                     // initiate I2C buss
    rslt = bme280_setup(&dev);                       		// set up bme280 interface
    if (rslt != BME280_OK)
        {
            printf( "Failed to set up bme280 (code %+d).", rslt);
    
        }

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev);
	
	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&dev.settings);
    req_delay *= 1000;                                                  // seems the program need longer than it thinks. there seems to be some issue between mil sec and micro sec
    printf("Temperature, Pressure, Humidity\r\n");
    /* Continuously stream sensor data */
    while (1) {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
        /* Wait for the measurement to complete and print data @25Hz */
        dev.delay_us(req_delay, dev.intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if (rslt != BME280_OK)
        {
            printf( "Failed to get sensor data (code %+d).", rslt);
            break;
        }
        print_sensor_data(&comp_data);
    }
	vTaskDelete(NULL);
	//return rslt;
}
int8_t 	  stream_sensor_data_normal_tft()
{	
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	struct bme280_dev dev;                                  //Structure to hold bme interface pointers. used in bme280_setup(&dev), USED TO OPPERATE THE BME280 CHIP
    ESP_ERROR_CHECK(i2c_master_init());                     // initiate I2C buss
    rslt = bme280_setup(&dev);                       		// set up bme280 interface
    if (rslt != BME280_OK)
        {
            printf( "Failed to set up bme280 (code %+d).", rslt);
    
        }

	font_rotate = 0;
	text_wrap = 0;
	font_transparent = 0;
	font_forceFixed = 0;
	TFT_resetclipwin();

	image_debug = 0;

	char dtype[16];

	switch (tft_disp_type)
	{
	case DISP_TYPE_ILI9341:
		sprintf(dtype, "ILI9341");
		break;
	case DISP_TYPE_ILI9488:
		sprintf(dtype, "ILI9488");
		break;
	case DISP_TYPE_ST7789V:
		sprintf(dtype, "ST7789V");
		break;
	case DISP_TYPE_ST7735:
		sprintf(dtype, "ST7735");
		break;
	case DISP_TYPE_ST7735R:
		sprintf(dtype, "ST7735R");
		break;
	case DISP_TYPE_ST7735B:
		sprintf(dtype, "ST7735B");
		break;
	default:
		sprintf(dtype, "Unknown");
	}

	uint8_t disp_rot = LANDSCAPE;
	//_demo_pass = 0;
	gray_scale = 0;
	doprint = 1;				

	TFT_setRotation(disp_rot);
	disp_header("BME280 TFT DEMO");
	TFT_setFont(COMIC24_FONT, NULL);
	int tempy = TFT_getfontheight() + 4;
	_fg = TFT_ORANGE;
	TFT_print("ESP32", CENTER, (dispWin.y2 - dispWin.y1) / 2 - tempy);
	TFT_setFont(UBUNTU16_FONT, NULL);
	_fg = TFT_CYAN;
	TFT_print("BME280 Demo", CENTER, LASTY + tempy);
	tempy = TFT_getfontheight() + 4;
	TFT_setFont(DEFAULT_FONT, NULL);
	_fg = TFT_GREEN;
	sprintf(tmp_buff, "Read speed: %5.2f MHz", (float)max_rdclock / 1000000.0);
	TFT_print(tmp_buff, CENTER, LASTY + tempy);

	Wait(4000);

	while (1)
	{
		
		if (doprint)
		{
			if (disp_rot == 1)
				sprintf(tmp_buff, "LANDSCAPE");
			if (disp_rot == 2)
				sprintf(tmp_buff, "PORTRAIT FLIP");
			if (disp_rot == 3)
				sprintf(tmp_buff, "LANDSCAPE FLIP");
			if (disp_rot == 0)
				sprintf(tmp_buff, "PORTRAIT");
			printf("\r\n==========================================\r\nDisplay: %s: %s %d,%d %s\r\n\r\n",
				   dtype, tmp_buff, _width, _height, ((gray_scale) ? "Gray" : "Color"));
		}

		int x, y, n;
		uint32_t end_time;

		disp_header("INITIALISING");

		end_time = clock() + 500;			// was   GDEMO_TIME
		n = 0;
		while ((clock() < end_time) && (Wait(0)))
		{
			y = 4;
			for (int f = DEFAULT_FONT; f < FONT_7SEG; f++)
			{
				_fg = random_color();
				TFT_setFont(f, NULL);
				TFT_print("My Project", 4, y);
				y += TFT_getfontheight() + 4;
				n++;
			}
		}
		sprintf(tmp_buff, "%d STRINGS", n);
		update_header(NULL, tmp_buff);
		Wait(-1000);					// was  -GDEMO_INFO_TIME


		/* Recommended mode of operation: Indoor navigation */
		dev.settings.osr_h = BME280_OVERSAMPLING_1X;
		dev.settings.osr_p = BME280_OVERSAMPLING_16X;
		dev.settings.osr_t = BME280_OVERSAMPLING_2X;
		dev.settings.filter = BME280_FILTER_COEFF_16;
		dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

		settings_sel = BME280_OSR_PRESS_SEL;
		settings_sel |= BME280_OSR_TEMP_SEL;
		settings_sel |= BME280_OSR_HUM_SEL;
		settings_sel |= BME280_STANDBY_SEL;
		settings_sel |= BME280_FILTER_SEL;

		rslt = bme280_set_sensor_settings(settings_sel, &dev);
		rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
			 
		disp_header("Getting DATA");
	//	printf("Temperature, Pressure, Humidity\r\n");
		while (1) 
		{
			/* Delay while the sensor completes a measurement */
			dev.delay_us(70000, dev.intf_ptr);
			rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
			if (rslt != BME280_OK)
			{
				printf( "Failed to get sensor data (code %+d).\n", rslt);
				break;
			}
		//	dew_from_humidity(&comp_data);
		//printf("Free Heap size: %d \n",xPortGetFreeHeapSize());
			print_sensor_data(&comp_data);
		//	Wait(-GDEMO_INFO_TIME);
		
		}
		//vTaskDelete(NULL);
		return rslt;	

	}
}
void 	  print_chip_info()
{
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

/* Save the Max/Min temperatures in NVS
   by first reading and then comparing
   the number that has been saved previously
   to determin if a write is necessary.
   Return an error if anything goes wrong
   during this process.
 */
esp_err_t save_limits_temperature(void)
{
	nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    int32_t Smax_temp = 0; // value will default to 0, if not set yet in NVS
	int32_t Smin_temp = 0;

    err = nvs_get_i32(my_handle, "Smax_temp", &Smax_temp);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
	if( Smax_temp < GmaxTemp )
	{
		// Write
		Smax_temp = GmaxTemp;
		err = nvs_set_i32(my_handle, "Smax_temp", Smax_temp);
		if (err != ESP_OK) return err;

		// Commit written value.
		// After setting any values, nvs_commit() must be called to ensure changes are written
		// to flash storage. Implementations may write to storage at other times,
		// but this is not guaranteed.
		err = nvs_commit(my_handle);
		if (err != ESP_OK) return err;
	}
	if( Smax_temp > GmaxTemp )
	{
		GmaxTemp = Smax_temp;
	}

	err = nvs_get_i32(my_handle, "Smin_temp", &Smin_temp);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
	if( Smin_temp > GminTemp )
	{
		// Write
		Smin_temp = GminTemp;
		err = nvs_set_i32(my_handle, "Smin_temp", Smin_temp);
		if (err != ESP_OK) return err;

		// Commit written value.
		// After setting any values, nvs_commit() must be called to ensure changes are written
		// to flash storage. Implementations may write to storage at other times,
		// but this is not guaranteed.
		err = nvs_commit(my_handle);
		if (err != ESP_OK) return err;
	}
	if( Smin_temp < GminTemp && Smin_temp != 0 )
	{
		GminTemp = Smin_temp;
	}

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

/* Save the number of module restarts in NVS
   by first reading and then incrementing
   the number that has been saved previously.
   Return an error if anything goes wrong
   during this process.
 */
esp_err_t save_restart_counter(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(my_handle, "restart_conter", &restart_counter);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // Write
    restart_counter++;
    err = nvs_set_i32(my_handle, "restart_conter", restart_counter);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}
/* Save new run time value in NVS
   by first reading a table of previously saved values
   and then adding the new value at the end of the table.
   Return an error if anything goes wrong
   during this process.
 */
esp_err_t save_run_time(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read the size of memory space required for blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "run_time", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // Read previously saved blob if available
    uint32_t* run_time = malloc(required_size + sizeof(uint32_t));
    if (required_size > 0) {
        err = nvs_get_blob(my_handle, "run_time", run_time, &required_size);
        if (err != ESP_OK) {
            free(run_time);
            return err;
        }
    }

    // Write value including previously saved blob if available
    required_size += sizeof(uint32_t);
    run_time[required_size / sizeof(uint32_t) - 1] = xTaskGetTickCount() * portTICK_PERIOD_MS;
    err = nvs_set_blob(my_handle, "run_time", run_time, required_size);
    free(run_time);

    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}
/* Read from NVS and print restart counter
   and the table with run times.
   Return an error if anything goes wrong
   during this process.
 */
esp_err_t print_what_saved(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read restart counter
    int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(my_handle, "restart_conter", &restart_counter);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    printf("Restart counter = %d\n", restart_counter);

    // Read run time blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, "run_time", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    printf("Run time:\n");
    if (required_size == 0) {
        printf("Nothing saved yet!\n");
    } else {
        uint32_t* run_time = malloc(required_size);
        err = nvs_get_blob(my_handle, "run_time", run_time, &required_size);
        if (err != ESP_OK) {
            free(run_time);
            return err;
        }
        for (int i = 0; i < required_size / sizeof(uint32_t); i++) {
            printf("%d: %d\n", i + 1, run_time[i]);
        }
        free(run_time);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

void app_main(void)
{
	// ======== DISPLAY INITIALIZATION  =========
	esp_err_t ret;
	// ======== SET GLOBAL VARIABLES ==========================

	// ========= DISPLAY TYPE ==========================================
	// ==== Set display type                         =====
	tft_disp_type = DEFAULT_DISP_TYPE;
	//tft_disp_type = DISP_TYPE_ILI9341;
	//tft_disp_type = DISP_TYPE_ILI9488;
	//tft_disp_type = DISP_TYPE_ST7735B;
	// ===================================================

	// ========= DISPLAY RESOLUTION ==========================================
	// === Set display resolution if NOT using default ===
	// === DEFAULT_TFT_DISPLAY_WIDTH &                 ===
	// === DEFAULT_TFT_DISPLAY_HEIGHT                  ===
	_width = DEFAULT_TFT_DISPLAY_WIDTH;   // smaller dimension
	_height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
	//_width = 128;  // smaller dimension
	//_height = 160; // larger dimension
	// ===================================================

	// ========== SPI CLOCK =========================================
	// ==== Set maximum spi clock for display read    ====
	//      operations, function 'find_rd_speed()'    ====
	//      can be used after display initialization  ====
	max_rdclock = 8000000;
	// ===================================================

	// =========== TFT PINS INITILISATION =========================================================
	// === Pins MUST be initialized before SPI interface initialization ===
	// ====================================================================
	TFT_PinsInit();

	// ====  CONFIGURE SPI DEVICES(s)  ====================================================================================
	spi_lobo_device_handle_t spi;
	spi_lobo_bus_config_t buscfg = {
		.miso_io_num = PIN_NUM_MISO, // set SPI MISO pin
		.mosi_io_num = PIN_NUM_MOSI, // set SPI MOSI pin
		.sclk_io_num = PIN_NUM_CLK,  // set SPI CLK pin
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 6 * 1024,
	};
	spi_lobo_device_interface_config_t devcfg = {
		.clock_speed_hz = 8000000,		   // Initial clock out at 8 MHz
		.mode = 0,						   // SPI mode 0
		.spics_io_num = -1,				   // we will use external CS pin
		.spics_ext_io_num = PIN_NUM_CS,	// external CS pin
		.flags = LB_SPI_DEVICE_HALFDUPLEX, // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
	};

#if USE_TOUCH == TOUCH_TYPE_XPT2046
	spi_lobo_device_handle_t tsspi = NULL;

	spi_lobo_device_interface_config_t tsdevcfg = {
		.clock_speed_hz = 2500000,   //Clock out at 2.5 MHz
		.mode = 0,					 //SPI mode 0
		.spics_io_num = PIN_NUM_TCS, //Touch CS pin
		.spics_ext_io_num = -1,		 //Not using the external CS
									 //.command_bits=8,                        //1 byte command
	};
#elif USE_TOUCH == TOUCH_TYPE_STMPE610
	spi_lobo_device_handle_t tsspi = NULL;

	spi_lobo_device_interface_config_t tsdevcfg = {
		.clock_speed_hz = 1000000,   //Clock out at 1 MHz
		.mode = STMPE610_SPI_MODE,   //SPI mode 0
		.spics_io_num = PIN_NUM_TCS, //Touch CS pin
		.spics_ext_io_num = -1,		 //Not using the external CS
		.flags = 0,
	};
#endif

	// ====================================================================================================================

	vTaskDelay(500 / portTICK_RATE_MS);
	printf("\r\n==============================\r\n");
	printf("BME280/TFT SENSOR DEMO\r\n");
	printf("==============================\r\n");
	printf("Pins used: miso=%d, mosi=%d, sck=%d, cs=%d\r\n", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
#if USE_TOUCH > TOUCH_TYPE_NONE
	printf(" Touch CS: %d\r\n", PIN_NUM_TCS);
#endif
	printf("==============================\r\n\r\n");

	// ==================================================================
	// ==== Initialize the SPI bus and attach the LCD to the SPI bus ====

	ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
	assert(ret == ESP_OK);
	printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
	disp_spi = spi;

	// ==== Test select/deselect ====
	ret = spi_lobo_device_select(spi, 1);
	assert(ret == ESP_OK);
	ret = spi_lobo_device_deselect(spi);
	assert(ret == ESP_OK);

	printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
	printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");

#if USE_TOUCH > TOUCH_TYPE_NONE
	// =====================================================
	// ==== Attach the touch screen to the same SPI bus ====

	ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &tsdevcfg, &tsspi);
	assert(ret == ESP_OK);
	printf("SPI: touch screen device added to spi bus (%d)\r\n", SPI_BUS);
	ts_spi = tsspi;

	// ==== Test select/deselect ====
	ret = spi_lobo_device_select(tsspi, 1);
	assert(ret == ESP_OK);
	ret = spi_lobo_device_deselect(tsspi);
	assert(ret == ESP_OK);

	printf("SPI: attached TS device, speed=%u\r\n", spi_lobo_get_speed(tsspi));
#endif

	// ================================
	// ==== Initialize the Display ====

	printf("SPI: display init...\r\n");
	TFT_display_init();
	printf("OK\r\n");
#if USE_TOUCH == TOUCH_TYPE_STMPE610
	stmpe610_Init();
	vTaskDelay(10 / portTICK_RATE_MS);
	uint32_t tver = stmpe610_getID();
	printf("STMPE touch initialized, ver: %04x - %02x\r\n", tver >> 8, tver & 0xFF);
#endif

	// ---- Detect maximum read speed ----
	max_rdclock = find_rd_speed();
	printf("SPI: Max rd speed = %u\r\n", max_rdclock);

	// ==== Set SPI clock used for display operations ====
	spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
	printf("SPI: Changed speed to %u\r\n", spi_lobo_get_speed(spi));

	printf("\r\n---------------------\r\n");
	printf("BME280 demo started\r\n");
	printf("---------------------\r\n");

	font_rotate = 0;
	text_wrap = 0;
	font_transparent = 0;
	font_forceFixed = 0;
	gray_scale = 0;
	TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(PORTRAIT);
	TFT_setFont(DEFAULT_FONT, NULL);
	TFT_resetclipwin();

#ifdef CONFIG_EXAMPLE_USE_WIFI

	ESP_ERROR_CHECK(nvs_flash_init());

	// ===== Set time zone ======
	setenv("TZ", "CET-1CEST", 0);
	tzset();
	// ==========================

	disp_header("GET NTP TIME");

	time(&time_now);
	tm_info = localtime(&time_now);

	// Is time set? If not, tm_year will be (1970 - 1900).
	if (tm_info->tm_year < (2016 - 1900))
	{
		ESP_LOGI(tag, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
		_fg = TFT_CYAN;
		TFT_print("Time is not set yet", CENTER, CENTER);
		TFT_print("Connecting to WiFi", CENTER, LASTY + TFT_getfontheight() + 2);
		TFT_print("Getting time over NTP", CENTER, LASTY + TFT_getfontheight() + 2);
		_fg = TFT_YELLOW;
		TFT_print("Wait", CENTER, LASTY + TFT_getfontheight() + 2);
		if (obtain_time())
		{
			_fg = TFT_GREEN;
			TFT_print("System time is set.", CENTER, LASTY);
		}
		else
		{
			_fg = TFT_RED;
			TFT_print("ERROR.", CENTER, LASTY);
		}
		time(&time_now);
		update_header(NULL, "");
		Wait(-2000);
	}
#endif
/*
	disp_header("File system INIT");
	_fg = TFT_CYAN;
	TFT_print("Initializing SPIFFS...", CENTER, CENTER);
	// ==== Initialize the file system ====
	printf("\r\n\n");
	vfs_spiffs_register();
	if (!spiffs_is_mounted)
	{
		_fg = TFT_RED;
		TFT_print("SPIFFS not mounted !", CENTER, LASTY + TFT_getfontheight() + 2);
	}
	else
	{
		_fg = TFT_GREEN;
		TFT_print("SPIFFS Mounted.", CENTER, LASTY + TFT_getfontheight() + 2);
	}
	Wait(-2000);
*/


// =============== START OF MY INITILISATIONS ===================================================

// Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
	{
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );


// =============== WS280 INITILISATION ===================================================
	// ==== Initialize the WS280 led/pixel chain ====
   rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_LED_RMT_TX_GPIO, LED_RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
   // led_strip_t *strip 
   strip = led_strip_new_rmt_ws2812(&strip_config);
  
    if (!strip) 
	{
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
	
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Test");
	
	uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

	for (int i = 0; i < CONFIG_STRIP_LED_NUMBER; i++)
	{
		for (int j = i; j < CONFIG_STRIP_LED_NUMBER; j += 1)
		{
			// Write RGB values to strip driver
			ESP_ERROR_CHECK(strip->set_pixel(strip, j, 255, 0, 0));
		}
		// Flush RGB values to LEDs
		ESP_ERROR_CHECK(strip->refresh(strip, 100)); 
		vTaskDelay(pdMS_TO_TICKS(LED_CHASE_SPEED_MS));
		strip->clear(strip, 50);
		vTaskDelay(pdMS_TO_TICKS(LED_CHASE_SPEED_MS));
	}

	print_chip_info();                                      // print usefull ESP chip info

//	xTaskCreate(&stream_sensor_data_normal_tft, "BME280 + WS280 + LCD",  2048, NULL, 5, NULL);
//	xTaskCreate(&stream_sensor_data_normal_mode, "BME280 + WS280 + LCD",  2048, NULL, 6, NULL);
//	xTaskCreate(&stream_sensor_data_forced_mode, "BME280 + WS280 + LCD",  2048, NULL, 6, NULL);
	
	err = print_what_saved();
    if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

    err = save_restart_counter();
    if (err != ESP_OK) printf("Error (%s) saving restart counter to NVS!\n", esp_err_to_name(err));

// To convert the bme280 functions back (NON task version) 
// Replace the return type of the function with int8_t
// Un comment return rslt and comment the line vTaskDelete(NULL) 
// Their the final two lines in each function
// then use bellow code to call function 	
	int8_t rslt = stream_sensor_data_normal_tft();
    //rslt = stream_sensor_data_forced_mode();
    if (rslt != BME280_OK)
    {
        printf( "Failed to stream sensor data (code %+d).\n", rslt);

		err = save_run_time();
        if (err != ESP_OK) printf("Error (%s) saving run time blob to NVS!\n", esp_err_to_name(err));

        exit(1);
    }

}

