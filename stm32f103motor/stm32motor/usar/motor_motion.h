#ifndef MOTOR_MOTION_H
#define MOTOR_MOTION_H

#include <stdint.h>
#include "mt6v10_modbus.h"

/* Output-shaft units. Initial empirical scale: 30 rpm at 100 * 50 us.
 * This is NOT a measured gearbox ratio or rotor-position calibration. */
#define MOTION_MAX_RPM                 350U
#define MOTION_MIN_RPM                 3U
#define MOTION_DEFAULT_RPM             30U
#define MOTION_MAX_DEGREES             360000L
#define MOTION_ENCODER_SIGN            (-1)
#define MOTION_BASE_RPM_X10            300L
#define MOTION_BASE_INTERVAL_TICKS     100L
#define MOTION_ACCEL_RPM_X10_PER_S     100L
#define MOTION_ENCODER_TIMEOUT_MS      100U
#define MOTION_POSITION_TOLERANCE_X10  20L /* degrees * 10; initial 2 deg band */

typedef enum {
    MOTION_IDLE = 0, MOTION_WAIT_STOP, MOTION_SPEED,
    MOTION_POSITION, MOTION_SETTLING, MOTION_DONE, MOTION_FAULT
} MotorMotionState;

typedef enum {
    MOTION_OK = 0, MOTION_ENCODER_LOST, MOTION_ENCODER_JUMP,
    MOTION_DRIVER_FAULT, MOTION_TRACKING_ERROR, MOTION_MOVE_TIMEOUT,
    MOTION_STOP_TIMEOUT
} MotorMotionFault;

typedef struct {
    MotorMotionState state;
    MotorMotionFault fault;
    uint8_t active;
    uint8_t position_mode;
    uint8_t feedback_ready;
    uint8_t reference_valid;
    int8_t direction;            /* requested speed direction, +1 = FWD */
    int8_t drive_direction;      /* 0 = remove PWM, +/-1 = apply PWM */
    uint16_t rpm_limit;          /* output rpm; also position speed limit */
    uint16_t interval_ticks;     /* 50 us; only used when drive_direction != 0 */
    int32_t speed_rpm_x10;       /* normalized encoder speed */
    int32_t ramp_rpm_x10;        /* signed reference after acceleration limit */
    int32_t position_deg_x10;   /* multi-turn, relative to software zero */
    int32_t target_deg_x10;
} MotorMotionStatus;

void MotorMotion_Init(void);
/* Foreground only. Feed every loop, including while legacy/CAN mode is active.
 * Commands validate the latest Update; no active recovery after a fault. */
void MotorMotion_Update(uint32_t now, const Mt6v10Status *encoder,
                        uint8_t driver_ready, uint8_t driver_fault);
void MotorMotion_GetStatus(MotorMotionStatus *status);
uint8_t MotorMotion_SetRpm(uint16_t rpm); /* 0 stops; 3..350, does not auto-start */
void MotorMotion_SetDirection(int8_t direction);
uint8_t MotorMotion_Start(void);         /* start continuous speed mode */
void MotorMotion_Stop(void);
uint8_t MotorMotion_Zero(void);          /* stopped + fresh + stationary */
uint8_t MotorMotion_Move(int32_t degrees, uint8_t relative); /* starts move */
uint8_t MotorMotion_ClearFault(void);    /* stopped + fresh + healthy driver */

#endif
