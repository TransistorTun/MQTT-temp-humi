/**
 * @file lvgl_helpers.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "sdkconfig.h"
#include "lvgl_helpers.h"
#include "esp_log.h"

#include "lvgl_tft/disp_spi.h"
#include "lvgl_touch/tp_spi.h"
#include "lvgl_spi_conf.h"
#include "lvgl/src/lv_core/lv_refr.h"

#define TAG "lvgl_helpers"

void lvgl_driver_init(void)
{
    printf("%s->Display hor size: %d, ver size: %d\n",TAG, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    printf("%s->Display buffer size: %d\n",TAG, DISP_BUF_SIZE);
    printf("%s->Initializing SPI master for display\n",TAG);
    lvgl_spi_driver_init(TFT_SPI_HOST,DISP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,SPI_BUS_MAX_TRANSFER_SZ, 1,-1, -1);
    disp_spi_add_device(TFT_SPI_HOST);
    disp_driver_init();

    printf("%s->Initializing SPI master for touch\n",TAG);
    // touch_driver_init();
}


bool lvgl_spi_driver_init(int host,
    int miso_pin, int mosi_pin, int sclk_pin,
    int max_transfer_sz,
    int dma_channel,
    int quadwp_pin, int quadhd_pin)
{
    assert((SPI_HOST <= host) && (VSPI_HOST >= host));
    const char *spi_names[] = {"SPI_HOST", "HSPI_HOST", "VSPI_HOST"};
    printf("%s->Configuring SPI host %s (%d)\n",TAG, spi_names[host], host);
    printf("%s->MISO pin: %d, MOSI pin: %d, SCLK pin: %d\n",TAG, miso_pin, mosi_pin, sclk_pin);
    printf("%s->Max transfer size: %d (bytes)\n",TAG, max_transfer_sz);
    spi_bus_config_t buscfg = {
        .miso_io_num = miso_pin,
	    .mosi_io_num = 23,
	    .sclk_io_num = 18,
	    .quadwp_io_num = quadwp_pin,
	    .quadhd_io_num = quadhd_pin,
        .max_transfer_sz = max_transfer_sz
    };
    printf("%s->Initializing SPI bus...\n",TAG);
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, dma_channel);
    assert(ret == ESP_OK);
    return ESP_OK != ret;
}

