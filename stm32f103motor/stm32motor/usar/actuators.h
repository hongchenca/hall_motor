#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call after MX_GPIO_Init(), MX_TIM3_Init() and MX_TIM4_Init().
 * Starts TIM3_CH4 with the brushed motor stopped and TIM4_CH3 at 1500 us.
 * Call setters from one execution context after successful initialization.
 */
HAL_StatusTypeDef Actuators_Init(void);

/* PB8 / TIM4_CH3, KST A08, 1 us per count. Clamped to 900..2100 us.
 * The pulse range must also suit the connected servo's mechanical limits.
 */
void Servo_SetPulseUs(uint16_t pulse_us);

/* PB1 / TIM3_CH4: -1000..1000 permille, clamped to that range.
 * Positive: PA4 IN1 high, PA5 IN2 low. Negative: the opposite.
 * Zero: PWM = 0 and both direction pins low.
 */
void BrushedMotor_SetDuty(int16_t duty_per_mille);

/* Stops only the brushed motor. The servo holds its last commanded pulse. */
void Actuators_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
