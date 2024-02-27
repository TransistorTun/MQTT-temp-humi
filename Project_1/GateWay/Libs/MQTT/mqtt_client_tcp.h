#ifndef MQTT_CLIENT_H_
#define MQTT_CLIENT_H_

#include <stdint.h>
#include "mqtt_client.h"

typedef struct 
{
    esp_mqtt_client_handle_t client;
}MQTT_Client_Data_t;


void mqtt_app_start(MQTT_Client_Data_t *MQTT_Client, char *url);

#endif