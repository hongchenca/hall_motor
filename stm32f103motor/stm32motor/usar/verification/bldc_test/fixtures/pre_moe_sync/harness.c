/* Peripheral/HAL boundary only. The algorithm is compiled from ../../bldc_motor.c. */
#include "bldc_motor.h"
#include "motor_config.h"
#include "adc.h"
#include "tim.h"
#include <string.h>

TIM_TypeDef mock_tim1, mock_tim2;
ADC_TypeDef mock_adc1;
GPIO_TypeDef mock_gpioa, mock_gpiob;
TIM_HandleTypeDef htim1, htim2;
ADC_HandleTypeDef hadc1;
DMA_Channel_TypeDef mock_dma_channel;
DMA_HandleTypeDef mock_dma;
static volatile uint16_t *mock_adc_buffer;
uint32_t mock_dma_length, mock_pa2_sample;
uint32_t mock_now, mock_subtick, mock_primask;
uint32_t mock_hal_timer_result, mock_hal_timer_it_result;
uint32_t mock_hal_calibration_result, mock_hal_adc_result;
uint32_t test_snapshot[23];

uint32_t Mock_GetPrimask(void) { return mock_primask; }
void Mock_SetPrimask(uint32_t key) { mock_primask = key; }
uint32_t HAL_GetTick(void) { return mock_now; }
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer)
{
    (void)timer;
    return (HAL_StatusTypeDef)mock_hal_timer_result;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
    (void)timer;
    return (HAL_StatusTypeDef)mock_hal_timer_it_result;
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *adc)
{
    (void)adc;
    return (HAL_StatusTypeDef)mock_hal_calibration_result;
}
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *adc, uint32_t *data, uint32_t length)
{
    mock_adc_buffer = (volatile uint16_t *)data;
    mock_dma_length = length;
    mock_dma_channel.CCR = 0xFU; /* Enable, complete, half-transfer and error IRQs. */
    adc->Instance->CR2 |= ADC_CR2_SWSTART;
    return (HAL_StatusTypeDef)mock_hal_adc_result;
}
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub)
{
    (void)irq; (void)preempt; (void)sub;
}
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
void HAL_NVIC_DisableIRQ(int irq) { (void)irq; }
void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; }

void Test_SetHall(uint32_t hall)
{
    mock_gpioa.IDR = (mock_gpioa.IDR & ~0xC0U) |
        ((hall & 1U) << 6) | (((hall >> 1) & 1U) << 7);
    mock_gpiob.IDR = (mock_gpiob.IDR & ~1U) | ((hall >> 2) & 1U);
}

void Test_SetBreak(uint32_t active)
{
    if (active != 0U) {
        mock_gpiob.IDR &= ~GPIO_PIN_12;
        if ((mock_tim1.BDTR & TIM_BDTR_BKE) != 0U) {
            mock_tim1.BDTR &= ~TIM_BDTR_MOE;
            mock_tim1.SR |= TIM_SR_BIF;
        }
    } else {
        mock_gpiob.IDR |= GPIO_PIN_12;
    }
}

void Test_SetBkinPin(uint32_t high)
{
    if (high != 0U) {
        mock_gpiob.IDR |= GPIO_PIN_12;
    } else {
        mock_gpiob.IDR &= ~GPIO_PIN_12;
    }
}

uint32_t Test_ResetWithBkin(uint32_t bkin_high)
{
    memset(&mock_tim1, 0, sizeof(mock_tim1));
    memset(&mock_tim2, 0, sizeof(mock_tim2));
    memset(&mock_adc1, 0, sizeof(mock_adc1));
    mock_gpioa.IDR = 0U;
    mock_gpiob.IDR = 0U;
    Test_SetBkinPin(bkin_high);
    Test_SetHall(1U);
    htim1.Instance = &mock_tim1;
    htim2.Instance = &mock_tim2;
    hadc1.Instance = &mock_adc1;
    hadc1.DMA_Handle = &mock_dma;
    mock_dma.Instance = &mock_dma_channel;
    mock_dma_channel.CCR = 0U;
    mock_adc_buffer = NULL;
    mock_dma_length = 0U;
    mock_pa2_sample = 1234U;
    mock_tim1.ARR = MOTOR_PWM_TICKS - 1U;
    mock_tim1.CCMR2 = 0xA500U; /* Detect accidental destruction of channel 4. */
    mock_tim1.BDTR = TIM_BDTR_AOE | TIM_BDTR_BKE;
    mock_now = 0U;
    mock_subtick = 0U;
    mock_primask = 0U;
    mock_hal_timer_result = 0U;
    mock_hal_timer_it_result = 0U;
    mock_hal_calibration_result = 0U;
    mock_hal_adc_result = 0U;
    return (uint32_t)Bldc_Init();
}

uint32_t Test_Reset(void) { return Test_ResetWithBkin(1U); }

void Test_SetTime(uint32_t now) { mock_now = now; }

/* Emulate DMA's writes to the actual buffer supplied by the application.
 * Leave DR containing rank 2, with EOC acknowledged by the DMA read. */
void Test_CompleteAdc(uint32_t pa1, uint32_t pa2)
{
    if ((mock_adc1.CR2 & ADC_CR2_SWSTART) == 0U ||
        mock_adc_buffer == NULL || mock_dma_length != 2U) {
        return;
    }
    mock_adc_buffer[0] = (uint16_t)pa1;
    mock_adc_buffer[1] = (uint16_t)pa2;
    mock_adc1.DR = pa2;
    mock_adc1.SR &= ~ADC_SR_EOC;
    mock_adc1.CR2 &= ~ADC_CR2_SWSTART;
    HAL_ADC_ConvCpltCallback(&hadc1);
}

void Test_HalfAdc(uint32_t pa1)
{
    if (mock_adc_buffer != NULL) {
        mock_adc_buffer[0] = (uint16_t)pa1;
    }
}

void Test_DmaError(void)
{
    HAL_ADC_ErrorCallback(&hadc1);
}

/* sample == 0xFFFF means the DMA has not completed a pair. */
void Test_One(uint32_t sample, uint32_t hall)
{
    Test_SetHall(hall);
    if (sample != 0xFFFFU) {
        Test_CompleteAdc(sample, mock_pa2_sample);
    }
    Bldc_FastTick();
}

/* Tick time advances at 20 kHz. refresh == 0x10000 disables command refresh;
 * otherwise it is the signed 16-bit command refreshed every 100 ms. */
void Test_Advance(uint32_t count, uint32_t sample, uint32_t hall, uint32_t refresh)
{
    uint32_t i;
    for (i = 0U; i < count; ++i) {
        if ((refresh != 0x10000U) && (mock_subtick == 0U) &&
            ((mock_now % 100U) == 0U)) {
            Bldc_SetVoltageMv((int16_t)refresh, mock_now);
        }
        Test_One(sample, hall);
        if (++mock_subtick == 20U) {
            mock_subtick = 0U;
            mock_now++;
        }
    }
}

void Test_Snapshot(void)
{
    BldcStatus s;
    Bldc_GetStatus(&s);
    test_snapshot[0] = (uint32_t)s.state;
    test_snapshot[1] = (uint32_t)s.fault;
    test_snapshot[2] = s.vbus_adc;
    test_snapshot[3] = s.vbus_mv;
    test_snapshot[4] = s.duty_ticks;
    test_snapshot[5] = s.hall;
    test_snapshot[6] = s.voltage_ready;
    test_snapshot[7] = s.command_timed_out;
    test_snapshot[8] = mock_tim1.CCER;
    test_snapshot[9] = mock_tim1.CCMR1;
    test_snapshot[10] = mock_tim1.CCMR2;
    test_snapshot[11] = mock_tim1.BDTR;
    test_snapshot[12] = mock_tim1.CCR1;
    test_snapshot[13] = mock_tim1.CCR2;
    test_snapshot[14] = mock_tim1.CCR3;
    test_snapshot[15] = mock_tim1.DIER;
    test_snapshot[16] = mock_now;
    test_snapshot[17] = mock_primask;
    test_snapshot[18] = s.pa2_adc;
    test_snapshot[19] = mock_dma_length;
    test_snapshot[20] = mock_dma_channel.CCR;
    test_snapshot[21] = (mock_gpiob.IDR & GPIO_PIN_12) != 0U;
    test_snapshot[22] = (mock_tim1.SR & TIM_SR_BIF) != 0U;
}

/* No CRT is needed by Unicorn. Retain the actual module through --no_remove. */
void test_entry(void) { }
