#include "gpio.h"

void GPIO_Input_Init(gpio_num_t pin, GPIO_INIT_PULL_t gpio_res_pull)
{
    if(gpio_res_pull == GPIO_NO_PULL)
    {
        gpio_config_t io_conf = 
        {
            .pin_bit_mask   =   (1 << pin),
            .mode           =   GPIO_MODE_INPUT,
            .pull_up_en     =   !gpio_res_pull,
            .pull_down_en   =   !gpio_res_pull,
            .intr_type      =   GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }
    else
    {
        gpio_config_t io_conf = 
        {
            .pin_bit_mask   =   (1 << pin),
            .mode           =   GPIO_MODE_INPUT,
            .pull_up_en     =   !gpio_res_pull,
            .pull_down_en   =   gpio_res_pull,
            .intr_type      =   GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }
}

void GPIO_Output_Init(gpio_num_t io_pin)
{
    gpio_config_t io_config = 
    {
        .mode           = GPIO_MODE_OUTPUT,
        .pin_bit_mask   = (1 << io_pin),
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_config);
}
void GPIO_SetState(gpio_num_t io_pin, uint8_t state)
{
    gpio_set_level(io_pin, state);
}
uint8_t GPIO_GetState(gpio_num_t io_pin)
{
    return gpio_get_level(io_pin);
}