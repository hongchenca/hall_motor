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

#endif
