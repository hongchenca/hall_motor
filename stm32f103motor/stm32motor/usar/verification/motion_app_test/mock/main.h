#ifndef MOTION_APP_TEST_MAIN_H
#define MOTION_APP_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2,
    CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR; } TIM_TypeDef;
typedef struct { volatile uint32_t IDR, BRR; } GPIO_TypeDef;
typedef struct { volatile uint32_t MAPR; } AFIO_TypeDef;
typedef struct { volatile uint32_t CR; } RCC_TypeDef;
typedef struct { volatile uint32_t SR, DR; } USART_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } UART_HandleTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern TIM_TypeDef mock_tim1, mock_tim3, mock_tim4;
extern GPIO_TypeDef mock_gpioa, mock_gpiob;
extern AFIO_TypeDef mock_afio;
extern RCC_TypeDef mock_rcc;
extern USART_TypeDef mock_usart3;
#define TIM1 (&mock_tim1)
#define TIM3 (&mock_tim3)
#define TIM4 (&mock_tim4)
#define GPIOA (&mock_gpioa)
#define GPIOB (&mock_gpiob)
#define AFIO (&mock_afio)
#define RCC (&mock_rcc)
#define USART3 (&mock_usart3)
#define USART_SR_TXE (1U << 7)
#define GPIO_PIN_0 1U
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_12 (1U << 12)
#define TIM_BDTR_MOE (1U << 15)
#define TIM_BDTR_BKE (1U << 12)
#define TIM_BDTR_BKP (1U << 13)
#define TIM_SR_BIF (1U << 7)
#define TIM_DIER_BIE (1U << 7)
#define TIM_SR_UIF 1U
#define TIM_DIER_UIE 1U
#define RCC_CR_CSSON (1U << 19)
#define DC_IN1_Pin (1U << 4)
#define DC_IN2_Pin (1U << 5)
#define RS485_DIR_Pin (1U << 5)
#define RS485_DIR_GPIO_Port GPIOB
#define FAULT_LED_Pin (1U << 13)
#define FAULT_LED_GPIO_Port GPIOB
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
uint32_t Mock_GetPrimask(void);
void Mock_SetPrimask(uint32_t mask);
#define __get_PRIMASK() Mock_GetPrimask()
#define __set_PRIMASK(k) Mock_SetPrimask(k)
#define __disable_irq() Mock_SetPrimask(1)
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
#endif
