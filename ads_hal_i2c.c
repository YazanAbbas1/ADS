/**
 * @file ads_hal_i2c.c
 * @author Yazan Abbas (mhd.yazan.abbas3@gmail.com)
 * @brief Angular Displacement Sensor
 * @version 0.1
 * @date 2022-12-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ads_hal.h"
#include <stdint.h>

/* Hardware Specific Includes */
#include <stdio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "sdkconfig.h"
#include "driver/gpio.h"

#define PIN_SDA 18
#define PIN_CLK 19
#define I2C_RX_BUF_DISABLE 0
#define I2C_TX_BUF_DISABLE 0
#define I2C_PORT	I2C_NUM_1
#define ACK					0
#define NAK					1
#define ENABLE		ACK

static char * TAG = {"ADS"};

static void (*ads_read_callback)(uint8_t *);

static uint8_t read_buffer[ADS_TRANSFER_SIZE];

#define ADS_DEFAULT_ADDR		(0x12)			// Default I2C address of the ADS one axis sensor

static uint32_t ADS_RESET_PIN = 0;
static uint32_t ADS_INTERRUPT_PIN = 0;


static uint8_t _address = ADS_DEFAULT_ADDR;

volatile bool _ads_int_enabled = false;


/************************************************************************/
/*                        HAL Stub Functions                            */
/************************************************************************/
static inline void ads_hal_gpio_pin_write(uint8_t pin, uint8_t val);
static void ads_hal_pin_int_init(void);
static void ads_hal_i2c_init(void);

/**
 * @brief ADS data ready interrupt. Reads out packet from ADS and fires 
 *  		callback in ads.c
 */
void ads_hal_interrupt(void* arg)
{
	if(ads_hal_read_buffer(read_buffer, ADS_TRANSFER_SIZE) == ADS_OK)
		ads_read_callback(read_buffer);
	
	
}

/**
 * @brief Initializes the pin ADS_INTERRUPT_PIN as a falling edge pin change interrupt(1).
 *			Assign the interrupt service routine as ads_hal_interrupt(2). Enable pullup(3)
 *			Enable interrupt(4)	
 */
static void ads_hal_pin_int_init(void)
{
    const gpio_config_t GPIOConfig = {
        .pin_bit_mask = ADS_INTERRUPT_PIN,
        .mode =  GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,//(3)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type =  GPIO_INTR_NEGEDGE,//(1) 
    };

 	ESP_ERROR_CHECK( gpio_config(&GPIOConfig) );
    ESP_ERROR_CHECK( gpio_intr_enable(ADS_INTERRUPT_PIN) );//(4)
    ESP_ERROR_CHECK( gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3) );//(2)
    ESP_ERROR_CHECK( gpio_isr_handler_add(ADS_INTERRUPT_PIN, ads_hal_interrupt, (void*) ADS_INTERRUPT_PIN) );//(2)
}

/**
 * @brief Write pin to level of val	
 */
static inline void ads_hal_gpio_pin_write(uint8_t pin, uint8_t val)
{
	ESP_ERROR_CHECK( gpio_set_level(pin, val) );
}

/**
 * @brief Millisecond delay routine.
 */
void ads_hal_delay(uint16_t delay_ms)
{
	vTaskDelay(delay_ms / portTICK_PERIOD_MS);
}

/**
 * @brief Enable/Disable the pin change data ready interrupt
 *
 * @param enable		true = enable, false = disable
 */
void ads_hal_pin_int_enable(bool enable)
{
	_ads_int_enabled = enable;
	
	if(enable)
	{
		ESP_ERROR_CHECK( gpio_intr_enable(ADS_INTERRUPT_PIN) );
		ESP_ERROR_CHECK( gpio_isr_handler_add(ADS_INTERRUPT_PIN, ads_hal_interrupt, (void*) ADS_INTERRUPT_PIN) );
	}
	else
	{
		ESP_ERROR_CHECK( gpio_isr_handler_remove(ADS_INTERRUPT_PIN) );
		ESP_ERROR_CHECK( gpio_intr_disable(ADS_INTERRUPT_PIN) );
	}
}

/**
 * @brief Configure I2C bus, 7 bit address, 100kHz frequency enable clock stretching
 *			if available.
 */
static void ads_hal_i2c_init(void)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = PIN_SDA,
		.scl_io_num = PIN_CLK,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000,
	};

	esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
	if (ret != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_param_config failed: %d", ret);
		return;
	}

	ret = i2c_driver_install(
			I2C_PORT, //I2C interface number
			conf.mode, // device mode (Master / Slave)
			I2C_RX_BUF_DISABLE, // Read buffer state (not needed in master mode)
			I2C_TX_BUF_DISABLE, 	// Transmitte buffer state (not needed in master mode)
			ESP_INTR_FLAG_LOWMED 	// Interrupt flags.
		);
	if (ret != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_driver_install failed: %d", ret);
		return;
	}

}

/**
 * @brief Write buffer of data to the Angular Displacement Sensor
 *
 * @param buffer[in]	Write buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_write_buffer(uint8_t * buffer, uint8_t len)
{
		// Disable the interrupt, if interrupt is enabled
	if(_ads_int_enabled)
		ads_hal_pin_int_enable(false);
	esp_err_t ret = i2c_master_write_to_device(
		I2C_PORT,
		ADS_DEFAULT_ADDR, 
		buffer,
		len, 
		1000 / portTICK_PERIOD_MS
	);
	ESP_ERROR_CHECK(ret);
	if(_ads_int_enabled)
	{
		ads_hal_pin_int_enable(true);
		// Read data packet if interrupt was missed
		if( gpio_get_level(ADS_INTERRUPT_PIN) == 0)
		{
			if(ads_hal_read_buffer(read_buffer, ADS_TRANSFER_SIZE) == ADS_OK)
			{
				ads_read_callback(read_buffer);
			}
		}
	}
    return ret;
}

/**
 * @brief Read buffer of data from the Angular Displacement Sensor
 *
 * @param buffer[out]	Read buffer
 * @param len			Length of data to read in number of bytes.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
esp_err_t ads_hal_read_buffer(uint8_t * buffer, uint8_t len)
{
	esp_err_t ret = i2c_master_read_from_device(
		I2C_PORT,
		ADS_DEFAULT_ADDR,
		buffer, 
		len,
		1000 / portTICK_PERIOD_MS
	);
	ESP_ERROR_CHECK(ret);
    return ret;
}

/**
 * @brief Reset the Angular Displacement Sensor
 */
void ads_hal_reset(void)
{
	// Configure reset line as an output
	ESP_ERROR_CHECK( gpio_set_direction(ADS_RESET_PIN, GPIO_MODE_OUTPUT) );
	// Bring reset low for 10ms then release
	ads_hal_gpio_pin_write(ADS_RESET_PIN, 0);
	ads_hal_delay(10);
	ads_hal_gpio_pin_write(ADS_RESET_PIN, 1);
}

/**
 * @brief Initializes the hardware abstraction layer 
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_init(void (*callback)(uint8_t*), uint32_t reset_pin, uint32_t datardy_pin)
{
	// Copy pin numbers for reset and data ready to local variables
	ADS_RESET_PIN     = reset_pin;
	ADS_INTERRUPT_PIN = datardy_pin;
	
	// Set callback pointer
	ads_read_callback = callback;
	
	// Reset the ads
	ads_hal_reset();
	
	// Wait for ads to initialize
	ads_hal_delay(2000);
	
	// Configure and enable interrupt pin
	ads_hal_pin_int_init();
	
	// Initialize the I2C bus
	ads_hal_i2c_init();

	return ADS_OK;
}

/**
 * @brief Gets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu)
 * @return	uint8_t _address
 */
uint8_t ads_hal_get_address(void)
{
	return _address;
}

/**
 * @brief Sets the i2c address that the hal layer is addressing *	
 *				Used by device firmware update (dfu)
 *
 * @param address		i2c address hal to communicate with
 */
void ads_hal_set_address(uint8_t address)
{
	_address = address;
}
