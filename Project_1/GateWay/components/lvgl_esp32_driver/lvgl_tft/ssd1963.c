/**
 * @file SSD1963.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ssd1963.h"
#include "driver/gpio.h"
#include "disp_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
// DEFINE I/O
#define SSD1963_PIN_CS      GPIO_NUM_33
#define SSD1963_PIN_DC      GPIO_NUM_15
#define SSD1963_PIN_WR      GPIO_NUM_4
#define SSD1963_PIN_RD      GPIO_NUM_21
#define SSD1963_PIN_RESET   GPIO_NUM_32
#define SSD1963_PIN_LATCH   GPIO_NUM_5
// #define SSD1963_PIN_BL      GPIO_NUM_15
// #define SSD1963_PIN_TE      GPIO_NUM_33
#define ILI9341_WIDTH  480
#define ILI9341_HEIGHT 272

// DEFINE 
#define PIN_HIGH(PIN)   gpio_set_level(PIN, 1)
#define PIN_LOW(PIN)    gpio_set_level(PIN, 0)

#define SSD1963_HOR_RES 800
#define SSD1963_VER_RES 480
#define SSD1963_HT 531
#define SSD1963_HPS 43
#define SSD1963_LPS 8
#define SSD1963_VT 288
#define SSD1963_VPS 12
#define SSD1963_HPW 10
#define SSD1963_VPW 10
#define SSD1963_FPS 4
#define TAG "SSD1963"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ssd1963_send_cmd(uint8_t cmd);
static void ssd1963_send_data(uint8_t data);
static void ssd1963_send_color(void * data);
static void ssd1963_write_bus(uint8_t data, uint8_t length);
static void ssd1963_io_init(void);
static void ssd1963_reset(void);
static void ssd1963_set_orientation(uint8_t orientation);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ssd1963_init(void)
{
    printf("%s->ssd1963 Initialization....\n", TAG);
    ssd1963_io_init();
    PIN_HIGH(SSD1963_PIN_CS);
    PIN_HIGH(SSD1963_PIN_RD);
    PIN_HIGH(SSD1963_PIN_WR);
    PIN_LOW(SSD1963_PIN_RESET);

    vTaskDelay(20 / portTICK_PERIOD_MS);
    PIN_HIGH(SSD1963_PIN_RESET);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    PIN_LOW(SSD1963_PIN_CS);
    // Send all the commands and parameter

    ssd1963_send_cmd(CMD_SET_PLL_MN); // PLL multiplier, set PLL clock to 120M
    ssd1963_send_data(0x23);          // N=0x36 for 6.5M, 0x23 for 10M crystal
    ssd1963_send_data(0x02);
    ssd1963_send_data(0x54);        

    ssd1963_send_cmd(CMD_PLL_START); // PLL enable
    ssd1963_send_data(0x01);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    ssd1963_send_cmd(CMD_PLL_START);
    ssd1963_send_data(0x03); // now, use PLL output as system clock

    vTaskDelay(10 / portTICK_PERIOD_MS);

    ssd1963_send_cmd(CMD_SOFT_RESET); // software reset

    vTaskDelay(100 / portTICK_PERIOD_MS);

    ssd1963_send_cmd(CMD_SET_PCLK); // PLL setting for PCLK, depends on resolution
    ssd1963_send_data(0x01);
    ssd1963_send_data(0x1F);
    ssd1963_send_data(0xFF);

    ssd1963_send_cmd(CMD_SET_PANEL_MODE); // LCD SPECIFICATION
	ssd1963_send_data(0x20);
	ssd1963_send_data(0x00);
	ssd1963_send_data(0x01);		//Set HDP	479
	ssd1963_send_data(0xDF);
	ssd1963_send_data(0x01);		//Set VDP	271
	ssd1963_send_data(0x0F);
	ssd1963_send_data(0x00);

    vTaskDelay(1 / portTICK_PERIOD_MS);

    ssd1963_send_cmd(CMD_SET_HOR_PERIOD);        // HSYNC
	ssd1963_send_data(0x02);		//Set HT	531
	ssd1963_send_data(0x13);
	ssd1963_send_data(0x00);		//Set HPS	8
	ssd1963_send_data(0x08);
	ssd1963_send_data(0x2B);		//Set HPW	43
	ssd1963_send_data(0x00);		//Set LPS	2
	ssd1963_send_data(0x02);
	ssd1963_send_data(0x00);

    ssd1963_send_cmd(CMD_SET_VER_PERIOD);        // VSYNC
	ssd1963_send_data(0x01);		//Set VT	288
	ssd1963_send_data(0x20);
	ssd1963_send_data(0x00);		//Set VPS	4
	ssd1963_send_data(0x04);
	ssd1963_send_data(0x0c);		//Set VPW	12
	ssd1963_send_data(0x00);		//Set FPS	2
	ssd1963_send_data(0x02);

    ssd1963_send_cmd(CMD_SET_GPIO_VAL);
    ssd1963_send_data(0x0F); //GPIO[3:0] out 1


    ssd1963_send_cmd(CMD_SET_GPIO_CONF);
	ssd1963_send_data(0x07);	    //GPIO3=input, GPIO[2:0]=output
	ssd1963_send_data(0x01);		//GPIO0 normal

    ssd1963_send_cmd(CMD_SET_ADDR_MODE); // rotation
    ssd1963_send_data(0x08);             // RGB=BGR

    ssd1963_send_cmd(CMD_SET_DATA_INTERFACE); // Pixel Data Interface Format
    ssd1963_send_data(0x03);                  // 8bit data

    ssd1963_send_cmd(CMD_SET_POST_PROC);
    ssd1963_send_data(0x40); // contrast value
    ssd1963_send_data(0x80); // brightness value
    ssd1963_send_data(0x40); // saturation value
    ssd1963_send_data(0x01); // Post Processor Enable

    vTaskDelay(1 / portTICK_PERIOD_MS);

    ssd1963_send_cmd(CMD_ON_DISPLAY); // display on

    ssd1963_send_cmd(CMD_SET_PWM_CONF); // set PWM for B/L
    ssd1963_send_data(0x06);
    ssd1963_send_data(0xF0);
    ssd1963_send_data(0x01);
    ssd1963_send_data(0xF0);
    ssd1963_send_data(0x00);
    ssd1963_send_data(0x00);

    ssd1963_send_cmd(CMD_SET_DBC_CONF);
    ssd1963_send_data(0x0D);

	ssd1963_send_cmd(0x2C);
}

void ssd1963_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, 
                        lv_color_t *color_p)

{
    gpio_set_level(SSD1963_PIN_CS, 0);
    // Column addresses
    ssd1963_send_cmd(0x2A);
    ssd1963_send_data((area->x1 >> 8) & 0xFF);
    ssd1963_send_data(area->x1 & 0xFF);
    ssd1963_send_data((area->x2 >> 8) & 0xFF);
    ssd1963_send_data(area->x2 & 0xFF);

    // Page addresses
    ssd1963_send_cmd(0x2B);
    ssd1963_send_data((area->y1 >> 8) & 0xFF);
    ssd1963_send_data(area->y1 & 0xFF);
    ssd1963_send_data((area->y2 >> 8) & 0xFF);
    ssd1963_send_data(area->y2 & 0xFF);
    // Memory write
    ssd1963_send_cmd(0x2C);

    gpio_set_level(SSD1963_PIN_DC, 1); /*Data mode*/

    uint16_t act_w = lv_area_get_width(area);
    for (uint16_t i = area->y1; i <= area->y2; i++)
    {
        for (uint16_t j = 0; j <= act_w; j++)
        {
            ssd1963_send_color((void *)color_p);
        }
    }
    // lv_disp_flush_ready(disp_drv);
    gpio_set_level(SSD1963_PIN_CS, 1);
}

void swap(uint16_t a, uint16_t b)
{
	uint16_t temp = 0x0000;

    temp = b;
    b = a;
    a = temp;
}
static void ILI9341_SetAddressWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	swap(x1, y1);
	swap(x2, y2);
	ssd1963_send_cmd(0x2a);
	ssd1963_send_data(x1>>8);
	ssd1963_send_data(x1);
	ssd1963_send_data(x2>>8);
	ssd1963_send_data(x2);
	ssd1963_send_cmd(0x2b);
	ssd1963_send_data(y1>>8);
	ssd1963_send_data(y1);
	ssd1963_send_data(y2>>8);
	ssd1963_send_data(y2);
	ssd1963_send_cmd(0x2c);
}
void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT)) return;
    if((x + w - 1) >= ILI9341_WIDTH) w = ILI9341_WIDTH - x;
    if((y + h - 1) >= ILI9341_HEIGHT) h = ILI9341_HEIGHT - y;
uint8_t temp[2] = {0, 0x1f};
    ILI9341_SetAddressWindow(x, y, x+w-1, y+h-1);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            ssd1963_write_bus(temp, 2);
        }
    }
}

void ILI9341_FillScreen(uint16_t color) {
    ILI9341_FillRectangle(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, color);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

// Initialize non-SPI GPIO
static void ssd1963_io_init(void)
{
    gpio_set_direction(SSD1963_PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(SSD1963_PIN_WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(SSD1963_PIN_RD, GPIO_MODE_OUTPUT);
    gpio_set_direction(SSD1963_PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(SSD1963_PIN_RESET, GPIO_MODE_OUTPUT);
}

static void ssd1963_write_bus(uint8_t data, uint8_t length)
{
    disp_wait_for_pending_transactions();
    
    disp_spi_send_data(&data, length);
    PIN_LOW(SSD1963_PIN_LATCH);
    PIN_HIGH(SSD1963_PIN_LATCH);
    
    PIN_LOW(SSD1963_PIN_WR);
    PIN_HIGH(SSD1963_PIN_WR);
}

static void ssd1963_send_color(void * data)
{
    disp_wait_for_pending_transactions();
    // Data Mode
    PIN_HIGH(SSD1963_PIN_DC);
    disp_spi_send_colors(data, 1);
}

static void ssd1963_send_cmd(uint8_t cmd)
{
    // Command Mode
    PIN_LOW(SSD1963_PIN_DC);
    ssd1963_write_bus(cmd, 1);
}

static void ssd1963_send_data(uint8_t data)
{
    // Data Mode
    PIN_HIGH(SSD1963_PIN_DC);
    ssd1963_write_bus(data, 1);
}

// Reset the display
static void ssd1963_reset(void)
{
    gpio_set_level(SSD1963_PIN_RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(SSD1963_PIN_RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void ssd1963_set_orientation(uint8_t orientation)
{
    const char *orientation_str[] = {"PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"};
    printf("%s->Display orientation: %s\n", TAG, orientation_str[orientation]);
    uint8_t data[] = {0x48, 0x88, 0x28, 0xE8};
    printf("%s->0x36 command value: 0x%02X\n", TAG, data[orientation]);
    ssd1963_send_cmd(CMD_SET_ADDR_MODE);
    ssd1963_send_data(data[orientation]);
}