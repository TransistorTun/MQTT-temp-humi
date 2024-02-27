#ifndef WIFI_H_
#define WIFI_H_

#include <stdint.h>

#define SCAN_LIST_SIZE 10
typedef enum
{
    CONNECT_OK = 1,
    CONNECT_FAIL,
    UNEXPECTED_EVENT,
}WIFI_Status_t;

uint8_t WIFI_Station_Init(uint8_t *ssid, uint8_t *password);
void WIFI_Scan(void);

#endif