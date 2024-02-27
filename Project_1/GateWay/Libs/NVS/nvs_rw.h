#ifndef NVS_H_
#define NVS_H_

#include <stdint.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

typedef enum
{
    UNSIGNED_INTEGER_8,
    UNSIGNED_INTEGER_16,
    UNSIGNED_INTEGER_32,
    UNSIGNED_INTEGER_64,
    SIGNED_INTEGER_8,
    SIGNED_INTEGER_16,
    SIGNED_INTEGER_32,
    SIGNED_INTEGER_64,
}TYPE_Data_t;

esp_err_t NVS_Init();
void NVS_Open(nvs_handle_t *my_handle);
void NVS_Read(nvs_handle_t *my_handle, TYPE_Data_t type, int64_t *data);
void NVS_Write(nvs_handle_t *my_handle, TYPE_Data_t type, int64_t *data);

#endif /* NVS_H_ */