

#ifndef _DRV_LED_H_
#define _DRV_LED_H_

#include "base_config.h"

typedef enum
{

    red,
    green,

} led_lists;

/**
 * @brief LED…Ë÷√
 *
 * @param ledx
 * @param status
 */
void drv_led_set(led_lists ledx, uint8_t status);

void drv_led_blink(led_lists ledx);

#endif
