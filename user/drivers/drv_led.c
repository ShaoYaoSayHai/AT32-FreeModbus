
#include "drv_led.h"

/**
 * @brief LED…Ë÷√
 *
 * @param ledx
 * @param status
 */
void drv_led_set(led_lists ledx, uint8_t status)
{
    if (ledx == green)
    {
        gpio_bits_write(GPIOA, GPIO_PINS_11, status);
    }
    if (ledx == red)
    {
        gpio_bits_write(GPIOA, GPIO_PINS_12, status);
    }
}

void drv_led_blink(led_lists ledx)
{
    static uint16_t blink_counters = 0;
    if ((blink_counters / 500) == 1)
    {
        drv_led_set(ledx, TRUE);
    }
    else if ((blink_counters / 500) == 2)
    {
        blink_counters = 0;
        drv_led_set(ledx, FALSE);
    }
    blink_counters++;
}
