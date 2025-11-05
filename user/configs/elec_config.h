/**
 * @file elec_config.h
 * @author Enzo
 * @brief  电磁阀控制配置
 * @version 0.1
 * @date 2025-11-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _ELEC_CONFIG_H_
#define _ELEC_CONFIG_H_

#include "base_config.h"

#define DRPORT_A (GPIOB)
#define DRPORT_B (GPIOB)
#define DRPIN_A (GPIO_PINS_4) // 打开驱动A向
#define DRPIN_B (GPIO_PINS_5) // 打开驱动B向

#endif
