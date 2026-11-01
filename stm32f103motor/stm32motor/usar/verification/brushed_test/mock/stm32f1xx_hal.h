#ifndef TEST_STM32F1XX_HAL_H
#define TEST_STM32F1XX_HAL_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2;
    volatile uint32_t CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR;
} TIM_TypeDef;

typedef struct { volatile uint32_t IDR, ODR, BSRR, BRR; } GPIO_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;

extern TIM_TypeDef mock_tim3, mock_tim4;
extern GPIO_TypeDef mock_gpioa, mock_gpiob;

#define TIM3 (&mock_tim3)
#define TIM4 (&mock_tim4)
#define GPIOA (&mock_gpioa)
#define GPIOB (&mock_gpiob)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define TIM_CHANNEL_3 0x8U
#define TIM_CHANNEL_4 0xCU
#define TIM_EGR_UG 1U

#define __HAL_TIM_SET_COMPARE(H, CH, VALUE) \
    do { \
        if ((CH) == TIM_CHANNEL_4) { (H)->Instance->CCR4 = (VALUE); } \
        else if ((CH) == TIM_CHANNEL_3) { (H)->Instance->CCR3 = (VALUE); } \
    } while (0)
#define __HAL_TIM_GET_AUTORELOAD(H) ((H)->Instance->ARR)

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state);

#endif
