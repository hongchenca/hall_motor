/* HAL boundary only. The tested control logic is compiled from ../../actuators.c. */
#include "actuators.h"
#include "main.h"
#include "tim.h"
#include <string.h>

TIM_TypeDef mock_tim3, mock_tim4;
GPIO_TypeDef mock_gpioa, mock_gpiob;
TIM_HandleTypeDef htim3, htim4;
uint32_t mock_pwm_start_calls, mock_pwm_stop_calls, mock_pwm_fail_call;
uint32_t mock_gpio_trace_count;
uint32_t mock_gpio_trace[8][4];
uint32_t test_snapshot[40];

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer;
    (void)channel;
    mock_pwm_start_calls++;
    return (mock_pwm_start_calls == mock_pwm_fail_call) ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer;
    (void)channel;
    mock_pwm_stop_calls++;
    return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        port->ODR |= pins;
    } else {
        port->ODR &= ~(uint32_t)pins;
    }
    if (mock_gpio_trace_count < 8U) {
        uint32_t *entry = mock_gpio_trace[mock_gpio_trace_count++];
        entry[0] = pins;
        entry[1] = (uint32_t)state;
        entry[2] = mock_tim3.CCR4;
        entry[3] = mock_tim3.EGR;
    }
}

uint32_t Test_Reset(void)
{
    memset(&mock_tim3, 0, sizeof(mock_tim3));
    memset(&mock_tim4, 0, sizeof(mock_tim4));
    memset(&mock_gpioa, 0, sizeof(mock_gpioa));
    memset(&mock_gpiob, 0, sizeof(mock_gpiob));
    memset(mock_gpio_trace, 0, sizeof(mock_gpio_trace));
    memset(test_snapshot, 0, sizeof(test_snapshot));
    htim3.Instance = &mock_tim3;
    htim4.Instance = &mock_tim4;
    mock_tim3.ARR = 3199U;
    mock_tim4.ARR = 19999U;
    mock_pwm_start_calls = 0U;
    mock_pwm_stop_calls = 0U;
    mock_pwm_fail_call = 0U;
    mock_gpio_trace_count = 0U;
    return 0U;
}

uint32_t Test_Init(void) { return (uint32_t)Actuators_Init(); }
void Test_Duty(int32_t duty) { BrushedMotor_SetDuty((int16_t)duty); }
void Test_Stop(void) { Actuators_Stop(); }
void Test_ClearTrace(void)
{
    memset(mock_gpio_trace, 0, sizeof(mock_gpio_trace));
    mock_gpio_trace_count = 0U;
}

void Test_Snapshot(void)
{
    uint32_t i;
    memset(test_snapshot, 0, sizeof(test_snapshot));
    test_snapshot[0] = mock_gpioa.ODR & (DC_IN1_Pin | DC_IN2_Pin);
    test_snapshot[1] = mock_tim3.CCR4;
    test_snapshot[2] = mock_tim4.CCR3;
    test_snapshot[3] = mock_tim3.ARR;
    test_snapshot[4] = mock_tim3.EGR;
    test_snapshot[5] = mock_pwm_start_calls;
    test_snapshot[6] = mock_pwm_stop_calls;
    test_snapshot[7] = mock_gpio_trace_count;
    for (i = 0U; i < 8U; ++i) {
        test_snapshot[8U + i * 4U] = mock_gpio_trace[i][0];
        test_snapshot[9U + i * 4U] = mock_gpio_trace[i][1];
        test_snapshot[10U + i * 4U] = mock_gpio_trace[i][2];
        test_snapshot[11U + i * 4U] = mock_gpio_trace[i][3];
    }
}

void test_entry(void) { }
