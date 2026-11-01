#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2;
    volatile uint32_t CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR;
} TIM_TypeDef;
typedef struct { volatile uint32_t SR, CR1, CR2, DR; } ADC_TypeDef;
typedef struct { volatile uint32_t IDR; } GPIO_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { volatile uint32_t CCR; } DMA_Channel_TypeDef;
typedef struct { DMA_Channel_TypeDef *Instance; } DMA_HandleTypeDef;
typedef struct { ADC_TypeDef *Instance; DMA_HandleTypeDef *DMA_Handle; } ADC_HandleTypeDef;

extern TIM_TypeDef mock_tim1, mock_tim2;
extern ADC_TypeDef mock_adc1;
extern GPIO_TypeDef mock_gpioa, mock_gpiob;
#define TIM1 (&mock_tim1)
#define TIM2 (&mock_tim2)
#define ADC1 (&mock_adc1)
#define GPIOA (&mock_gpioa)
#define GPIOB (&mock_gpiob)

#define GPIO_PIN_12 (1U << 12)
#define TIM_BDTR_MOE (1U << 15)
#define TIM_BDTR_AOE (1U << 14)
#define TIM_BDTR_BKE (1U << 12)
#define TIM_BDTR_OSSI (1U << 10)
#define TIM_BDTR_OSSR (1U << 11)
#define TIM_SR_BIF (1U << 7)
#define TIM_CR2_CCPC (1U << 0)
#define TIM_CR2_OIS1 (1U << 8)
#define TIM_CR2_OIS1N (1U << 9)
#define TIM_CR2_OIS2 (1U << 10)
#define TIM_CR2_OIS2N (1U << 11)
#define TIM_CR2_OIS3 (1U << 12)
#define TIM_CR2_OIS3N (1U << 13)
#define TIM_EGR_UG 1U
#define TIM_FLAG_UPDATE 1U
#define TIM_FLAG_BREAK TIM_SR_BIF
#define TIM_IT_BREAK (1U << 7)
#define TIM1_UP_IRQn 25
#define TIM1_BRK_IRQn 24
#define ADC_SR_EOC (1U << 1)
#define ADC_CR2_SWSTART (1U << 22)
#define DMA_IT_HT (1U << 2)
#define __HAL_DMA_DISABLE_IT(H, I) ((H)->Instance->CCR &= ~(I))
#define __ALIGN_BEGIN __align(4)
#define __ALIGN_END
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
/* Model TIM SR write-zero-to-clear behavior at the register boundary. */
#define __HAL_TIM_CLEAR_FLAG(H, F) ((H)->Instance->SR &= ~(F))
#define __HAL_TIM_DISABLE_IT(H, I) ((H)->Instance->DIER &= ~(I))
#define __HAL_TIM_ENABLE_IT(H, I) ((H)->Instance->DIER |= (I))
uint32_t Mock_GetPrimask(void);
void Mock_SetPrimask(uint32_t key);
#define __get_PRIMASK() Mock_GetPrimask()
#define __set_PRIMASK(key) Mock_SetPrimask(key)
#define __disable_irq() Mock_SetPrimask(1U)

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *adc, uint32_t *data, uint32_t length);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc);
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
#endif
