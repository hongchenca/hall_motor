#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call once after all CubeMX MX_*_Init functions. Outputs start at zero,
 * except the servo, which starts with a 1500 us pulse. */
HAL_StatusTypeDef MotorApp_Init(void);
void MotorApp_Process(void);

/* Immediate hardware output disable, usable even during partial startup.
 * Fatal handler only: resetting/reinitializing is required to resume. */
void MotorApp_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif
