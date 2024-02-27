// XPT2046.h
#ifndef XPT2046_H
#define XPT2046_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include <stdbool.h>
#include "lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/
#define XPT2046_IRQ			22
#define XPT2046_MOSI		23
#define XPT2046_MISO		19
#define XPT2046_CLK			18
#define XPT2046_CS			5

#define XPT2046_AVG			4
#define XPT2046_X_MIN		200
#define XPT2046_Y_MIN		200
#define XPT2046_X_MAX		1900
#define XPT2046_Y_MAX		1900
#define XPT2046_X_INV		1
#define XPT2046_Y_INV		1
#define XPT2046_XY_SWAP		0

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void xpt2046_init(void);
bool xpt2046_read(lv_indev_drv_t * drv, lv_indev_data_t * data);
uint16_t xpt2046_gpio_spi_read_reg(uint8_t reg);
void xpt2046_gpio_Write_Byte(uint8_t data);
uint16_t TP_Read_XOY(uint8_t xy);


uint8_t TP_Read_XY(uint16_t *x,uint16_t *y);
uint8_t TP_Read_XY2(uint16_t *x,uint16_t *y);
/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* XPT2046_H */
