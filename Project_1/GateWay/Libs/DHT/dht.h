#ifndef DHT_H
#define DHT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef enum
{
    DHT_CRC_ERROR = -1,
    DHT_TIMEOUT_ERROR,
    DHT_OK
} DHT11_Status_t;

typedef struct {
    float temperature; 
    float humidity;
    uint32_t io_pin;    
} DHT11_Data_t;


esp_err_t DHT11_Init(DHT11_Data_t *dht11_data, uint32_t io_pin);
uint8_t DHT11_Get_Data(DHT11_Data_t *dht11_data);

#endif /* DHT_H */