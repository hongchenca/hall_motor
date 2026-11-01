#include "actuators.h"
#include "main.h"
#include "tim.h"

#define SERVO_MIN_PULSE_US  900U
#define SERVO_MAX_PULSE_US  2100U
#define SERVO_MID_PULSE_US  1500U
#define BRUSHED_MAX_DUTY    1000

static uint8_t actuators_initialized = 0U;
static int8_t brushed_direction = 0;

static void BrushedMotor_ClearDrive(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);

    /* PWM uses a preloaded CCR4. Commit zero before touching IN1/IN2.
     * TIM3 is dedicated to this motor, so this update may reset its phase.
     */
    htim3.Instance->EGR = TIM_EGR_UG;
    HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin | DC_IN2_Pin,
                      GPIO_PIN_RESET);
    brushed_direction = 0;
}

HAL_StatusTypeDef Actuators_Init(void)
{
    HAL_StatusTypeDef status;

    if (actuators_initialized != 0U)
    {
        return HAL_OK;
    }

    if ((htim3.Instance != TIM3) || (htim4.Instance != TIM4))
    {
        return HAL_ERROR;
    }

    BrushedMotor_ClearDrive();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, SERVO_MID_PULSE_US);
    htim4.Instance->EGR = TIM_EGR_UG;

    status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    if (status != HAL_OK)
    {
        BrushedMotor_ClearDrive();
        (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
        return status;
    }

    actuators_initialized = 1U;
    return HAL_OK;
}

void Servo_SetPulseUs(uint16_t pulse_us)
{
    if (actuators_initialized == 0U)
    {
        return;
    }

    if (pulse_us < SERVO_MIN_PULSE_US)
    {
        pulse_us = SERVO_MIN_PULSE_US;
    }
    else if (pulse_us > SERVO_MAX_PULSE_US)
    {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_us);
}

void BrushedMotor_SetDuty(int16_t duty_per_mille)
{
    int8_t requested_direction;
    uint32_t magnitude;
    uint32_t period_counts;
    uint32_t compare;

    if (actuators_initialized == 0U)
    {
        return;
    }

    if (duty_per_mille > BRUSHED_MAX_DUTY)
    {
        duty_per_mille = BRUSHED_MAX_DUTY;
    }
    else if (duty_per_mille < -BRUSHED_MAX_DUTY)
    {
        duty_per_mille = -BRUSHED_MAX_DUTY;
    }

    if (duty_per_mille == 0)
    {
        BrushedMotor_ClearDrive();
        return;
    }

    requested_direction = (duty_per_mille > 0) ? 1 : -1;
    magnitude = (duty_per_mille > 0)
                    ? (uint32_t)duty_per_mille
                    : (uint32_t)(-(int32_t)duty_per_mille);

    if (requested_direction != brushed_direction)
    {
        BrushedMotor_ClearDrive();
        if (requested_direction > 0)
        {
            HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_SET);
        }
        brushed_direction = requested_direction;
    }

    period_counts = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
    compare = (magnitude * period_counts) / (uint32_t)BRUSHED_MAX_DUTY;
    /* ARR + 1 represents 100% duty in the configured PWM1 upcount mode. */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare);
}

void Actuators_Stop(void)
{
    if (actuators_initialized != 0U)
    {
        BrushedMotor_ClearDrive();
    }
}
