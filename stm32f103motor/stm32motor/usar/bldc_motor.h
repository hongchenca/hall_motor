#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include "main.h"

typedef enum {
    BLDC_FAULT_NONE = 0,
    BLDC_FAULT_HALL = 1,
    BLDC_FAULT_UNDERVOLTAGE = 2,
    BLDC_FAULT_OVERVOLTAGE = 3,
    /* Code 4 reserved: PA2 has no calibrated current/overcurrent mapping yet. */
    BLDC_FAULT_BREAK = 5,
    BLDC_FAULT_ADC = 6
} BldcFault;

typedef enum {
    BLDC_STOPPED = 0,
    BLDC_PRECHARGE,
    BLDC_RUNNING,
    BLDC_COASTING
} BldcState;

typedef struct {
    BldcState state;
    BldcFault fault;
    uint16_t vbus_adc;
    uint16_t pa2_adc; /* Latest PA2 raw sample (0..4095), no current scaling. */
    uint32_t vbus_mv;
    uint16_t duty_ticks;
    uint8_t hall;
    uint8_t voltage_ready;
    uint8_t command_timed_out;
    uint8_t open_loop_test;
    uint8_t commutation_step; /* 1..6 in the forward physical phase list; 0 off. */
    uint16_t commutation_interval_ticks; /* Current full step interval, 50 us. */
} BldcStatus;

typedef enum {
    BLDC_BREAK_NONE = 0,
    BLDC_BREAK_INIT_PIN_LOW = 1,
    BLDC_BREAK_TICK_PIN_LOW = 2,
    BLDC_BREAK_IRQ = 3,
    BLDC_BREAK_ENABLE_PRECHECK = 4,
    BLDC_BREAK_ENABLE_POSTCHECK = 5
} BldcBreakSource;

/* First Break fault snapshot, before outputs_off() changes the registers. */
typedef struct {
    BldcBreakSource source;
    uint32_t sr;
    uint32_t bdtr;
    uint32_t dier;
    uint8_t pb12;
    uint8_t hall;
    uint8_t state;
} BldcBreakDiagnostic;

HAL_StatusTypeDef Bldc_Init(void);
/* received_ms must be the receive timestamp, not a repeatedly refreshed loop time. */
void Bldc_SetVoltageMv(int16_t voltage_mv, uint32_t received_ms);
/* Time-driven mode only: 50 us units, clamped to configured bounds. Changes
 * apply at startup or the next phase boundary, never shorten an active step.
 * EmergencyStop restores the default interval; normal Hall mode ignores it. */
void Bldc_SetCommutationTicks(uint16_t ticks);
void Bldc_FastTick(void); /* exactly once per TIM1 update (50 us) */
void Bldc_BreakIrq(void);
/* Call at the TIM1 Break vector before HAL clears BIF; no UART in the ISR. */
void Bldc_CaptureBreakIrq(void);
void Bldc_GetBreakDiagnostic(BldcBreakDiagnostic *diagnostic);
void Bldc_GetStatus(BldcStatus *status);
/* No automatic recovery. Requires zero command and healthy supply/BKIN.
 * Hall validity is required only when the normal Hall controller is built. */
HAL_StatusTypeDef Bldc_ClearFault(void);
void Bldc_EmergencyStop(void);

#endif
