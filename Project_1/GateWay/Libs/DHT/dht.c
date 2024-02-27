#include "dht.h"
#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *DHT11_TAG = "DHT11";

esp_err_t DHT11_Init(DHT11_Data_t *dht11_data, uint32_t io_pin)
{
    dht11_data->io_pin=  io_pin;
    esp_rom_gpio_pad_select_gpio(dht11_data->io_pin);
    return ESP_OK;
}

static uint8_t MCU_DHT_Signal(DHT11_Data_t *dht11_data) 
{
    gpio_set_direction(dht11_data->io_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dht11_data->io_pin, 0); // pull down
    ets_delay_us(18000);      // MCU sends out start signal and pulls down voltage for at least 18ms to let DHT11 detect the signal
    gpio_set_level(dht11_data->io_pin, 1); // pull up
    ets_delay_us(30);             // MCU pulls up voltage and waits for DHT respone

    gpio_set_direction(dht11_data->io_pin, GPIO_MODE_INPUT);


    //CHECK RESPONE
    int64_t GetTick_us = esp_timer_get_time();
    while (!gpio_get_level(dht11_data->io_pin)) //DHT sends out response signal & keeps it for 80us
    { 
        if (esp_timer_get_time() - GetTick_us > 100)
            return DHT_TIMEOUT_ERROR;
    }

    GetTick_us = esp_timer_get_time();
    while (gpio_get_level(dht11_data->io_pin)) //DHT pulls up voltage and keeps it for 80us
    { 
        if (esp_timer_get_time() - GetTick_us > 100)
            return DHT_TIMEOUT_ERROR;
    }

    return DHT_OK;
}

static uint8_t DHT11_Read_Byte(DHT11_Data_t *dht11_data, uint8_t *value) 
{
    *value = 0;	
    for (int i = 8; i > 0; i--) 
    {
        *value <<= 1;

        int64_t timestamp_us = esp_timer_get_time();
        while (gpio_get_level(dht11_data->io_pin) == 0) //start to transmit 1-bit data (50us)
        { 
            if (esp_timer_get_time() - timestamp_us > 70)
                return DHT_TIMEOUT_ERROR;
        }

        ets_delay_us(40); 
        if (gpio_get_level(dht11_data->io_pin) == 1)  //if the pin is low => bit 0 ; if the pin is high => bit 1
        {
            *value |= 1;
        }

        timestamp_us = esp_timer_get_time();
        while (gpio_get_level(dht11_data->io_pin) == 1)  //wait for the pin to go low 
        {
            if (esp_timer_get_time() - timestamp_us > 50)
                return DHT_TIMEOUT_ERROR;
        }
    }
    return DHT_OK;
}

uint8_t DHT11_Get_Data(DHT11_Data_t *dht11_data) {
    uint8_t buf[4] = {0};
    uint8_t checksum;

    if (MCU_DHT_Signal(dht11_data) == DHT_TIMEOUT_ERROR) 
    {
        ESP_LOGE(DHT11_TAG, "No responses from DHT11!");
        return DHT_TIMEOUT_ERROR;
    }

    if ((DHT11_Read_Byte(dht11_data, &buf[0]) 
        && DHT11_Read_Byte(dht11_data, &buf[1]) 
        && DHT11_Read_Byte(dht11_data, &buf[2]) 
        &&DHT11_Read_Byte(dht11_data, &buf[3]) 
        && DHT11_Read_Byte(dht11_data, &checksum)) == DHT_TIMEOUT_ERROR) 
    {
        ESP_LOGE(DHT11_TAG, "Failed to read data from DHT11!");
        return DHT_TIMEOUT_ERROR;
    }

    //Data format: 8bit integral RH data + 8bit decimal RH data + 8bit integral T data +  
    //8bit decimal T data + 8bit check sum
    //If the data transmit is right, the checksum should be 
    //the last 8 bit of ""8bit integral RH data + 8bit decimal RH data + 8bit integral T data + 8bit decimal T data"
    if (checksum != buf[0] + buf[1] + buf[2] + buf[3]) 
    { 
        ESP_LOGE(DHT11_TAG, "Data from DHT11 is invalid!");
        return DHT_CRC_ERROR;
    }

    dht11_data->temperature = (float)buf[2] + (float)(buf[3] / 100.0);
    dht11_data->humidity = (float)buf[0] + (float)(buf[1] / 100.0);
    ESP_LOGI(DHT11_TAG, "Temperature: %f Humidity: %f", dht11_data->temperature, dht11_data->humidity);
    return DHT_OK;
}