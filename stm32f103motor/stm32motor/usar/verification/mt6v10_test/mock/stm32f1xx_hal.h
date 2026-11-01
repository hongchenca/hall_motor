#ifndef STM32F1XX_HAL_H
#define STM32F1XX_HAL_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

#define HAL_MAX_DELAY 0xFFFFFFFFU

#endif
