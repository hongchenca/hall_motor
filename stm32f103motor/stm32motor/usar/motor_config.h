#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* CAN protocol from the reference project; valid node addresses: 1..8. */
#define MOTOR_CAN_NODE_ID             1U
#define MOTOR_COMMAND_TIMEOUT_MS      500U

/* Original board PA1 divider: ADC=3385 at approximately 30 V (VDDA=3.3 V).
 * Recalibrate these constants if the divider or VDDA changes on the F103 PCB. */
#define MOTOR_VBUS_REFERENCE_ADC       3385U
#define MOTOR_VBUS_REFERENCE_MV        30000U
#define MOTOR_VBUS_UNDERVOLTAGE_ADC    1805U
#define MOTOR_VBUS_OVERVOLTAGE_ADC     3385U
#define MOTOR_VBUS_CONFIRM_MS          10U
#define MOTOR_ADC_FILTER_SAMPLES       64U
#define MOTOR_ADC_TIMEOUT_TICKS        40U

/* TIM1: 64 MHz / 3200 = 20 kHz. Original minimum running duty: 256/3200. */
#define MOTOR_PWM_TICKS                3200U
#define MOTOR_MIN_DUTY_TICKS           256U
#define MOTOR_MAX_DUTY_TICKS           3200U
#define MOTOR_PRECHARGE_TICKS          2000U  /* 100 ms, original low-side startup */
#define MOTOR_STALL_TIMEOUT_TICKS      65535U /* approximately 3.28 s */
#define MOTOR_REVERSE_COAST_TICKS      2000U  /* all outputs off before reversal */

/* Time-driven six-step bench test. Set to 0 for the original Hall controller.
 * Tick units below are 50 us. This is not a sensorless closed-loop controller. */
#ifndef MOTOR_OPEN_LOOP_TEST
#define MOTOR_OPEN_LOOP_TEST            1U
#endif
#define MOTOR_OPEN_LOOP_MAX_DUTY_PERCENT 100U
#define MOTOR_OPEN_LOOP_COMMUTATION_TICKS         100U /* 5 ms at 20 kHz */
#define MOTOR_OPEN_LOOP_MIN_COMMUTATION_TICKS       8U /* 400 us */
#define MOTOR_OPEN_LOOP_MAX_COMMUTATION_TICKS    2000U /* 100 ms */
#define MOTOR_OPEN_LOOP_MAX_DUTY_TICKS  \
    ((MOTOR_PWM_TICKS * MOTOR_OPEN_LOOP_MAX_DUTY_PERCENT) / 100U)

#if (MOTOR_OPEN_LOOP_TEST != 0U) && (MOTOR_OPEN_LOOP_TEST != 1U)
#error MOTOR_OPEN_LOOP_TEST_must_be_0_or_1
#endif
#if (MOTOR_OPEN_LOOP_MAX_DUTY_PERCENT == 0U) || \
    (MOTOR_OPEN_LOOP_MAX_DUTY_PERCENT > 100U) || \
    (MOTOR_OPEN_LOOP_MAX_DUTY_TICKS < MOTOR_MIN_DUTY_TICKS) || \
    (MOTOR_OPEN_LOOP_MAX_DUTY_TICKS > MOTOR_MAX_DUTY_TICKS)
#error Invalid_open_loop_duty_limit
#endif
#if (MOTOR_PRECHARGE_TICKS == 0U) || (MOTOR_PRECHARGE_TICKS > 65535U) || \
    (MOTOR_REVERSE_COAST_TICKS == 0U) || (MOTOR_REVERSE_COAST_TICKS > 65535U)
#error Invalid_open_loop_state_interval
#endif
#if (MOTOR_OPEN_LOOP_MIN_COMMUTATION_TICKS == 0U) || \
    (MOTOR_OPEN_LOOP_MAX_COMMUTATION_TICKS > 65535U) || \
    (MOTOR_OPEN_LOOP_MIN_COMMUTATION_TICKS > MOTOR_OPEN_LOOP_COMMUTATION_TICKS) || \
    (MOTOR_OPEN_LOOP_COMMUTATION_TICKS > MOTOR_OPEN_LOOP_MAX_COMMUTATION_TICKS)
#error Invalid_open_loop_commutation_interval
#endif

#endif
