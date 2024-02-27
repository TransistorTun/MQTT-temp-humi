#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_system.h"
// #include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "lvgl/lvgl.h"
#include "lvgl_helpers.h"

#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#include "ssd1963.h"

// #define DHT11_PIN   GPIO_NUM_5

// static uint8_t 			ssid[32] = "Khoa";
// static uint8_t 			pass[32] = "17042021";
// static char			url_mqtt[30] = "mqtt://172.20.10.5:1883";

// // static uint8_t 			ssid[32] = "TP-Link";
// // static uint8_t 			pass[32] = "khoaanbk21";

// static float temperature = 0, humidity = 0;
// DHT11_Data_t dht11_0;
// MQTT_Client_Data_t mqtt_client_0;

// static TaskHandle_t DHT11_task;

// void DHT11(void *arg)
// {
	// while (1)
	// {
	// 	DHT11_Get_Data(&dht11_0);
	// 	temperature = dht11_0.temperature;
	// 	humidity = dht11_0.humidity;
	// 	char str[5];
	// 	sprintf(str, "%.1f", temperature);
	// 	esp_mqtt_client_publish(mqtt_client_0.client, "dht11_tem", str, 0, 1, 0);
	// 	sprintf(str, "%.1f", humidity);
	// 	esp_mqtt_client_publish(mqtt_client_0.client, "dht11_hum", str, 0, 1, 0);
	// 	vTaskDelay(2000 / portTICK_PERIOD_MS);
	// }
// }

// void app_main(void)
// {
//     WIFI_Station_Init(ssid, pass);
//     mqtt_app_start(&mqtt_client_0, url_mqtt);
//     DHT11_Init(&dht11_0, DHT11_PIN);
//     xTaskCreate(DHT11, "DHT11", 1024 * 2, NULL, 10, &DHT11_task);
// }

/*********************
 *      DEFINES
 *********************/
#define TAG " LittlevGL Demo"
#define LV_TICK_PERIOD_MS 10

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
void guiTask(void *pvParameter);


void app_main() {
	printf("\r\nAPP %s is start!~\r\n", TAG);

	vTaskDelay(1000 / portTICK_PERIOD_MS);
	xTaskCreatePinnedToCore(guiTask, "gui", 4096*1, NULL, 0, NULL, 1);
    
}

static void lv_tick_task(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

//Creates a semaphore to handle concurrent call to lvgl stuff
//If you wish to call *any* lvgl function from other threads/tasks
//you should lock on the very same semaphore!
SemaphoreHandle_t xGuiSemaphore;

void guiTask(void *pvParameter) {
    
    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();
    lv_init();
    lvgl_driver_init(); 

    static lv_color_t buf1[DISP_BUF_SIZE];
    static lv_color_t buf2[DISP_BUF_SIZE];
    static lv_disp_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);


// #if CONFIG_LVGL_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
//     lv_indev_drv_t indev_drv;
//     lv_indev_drv_init(&indev_drv);
//     indev_drv.read_cb = touch_driver_read;
//     indev_drv.type = LV_INDEV_TYPE_POINTER;
//     lv_indev_drv_register(&indev_drv);
// #endif


    // const esp_timer_create_args_t periodic_timer_args = {
    //     .callback = &lv_tick_task,
    //     .name = "periodic_gui"
    // };
    // esp_timer_handle_t periodic_timer;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));
	// lv_demo_widgets();
    ILI9341_FillScreen(0);
    printf("test\n");
    // lv_demo_widgets();
    while (1) 
    {
		vTaskDelay(1);
		if (xSemaphoreTake(xGuiSemaphore, (TickType_t)10) == pdTRUE) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }
    vTaskDelete(NULL);
}