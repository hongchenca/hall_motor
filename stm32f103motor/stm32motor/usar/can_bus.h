#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call after MX_CAN_Init(), with a node ID from 1 through 8.
 * An already started bus returns HAL_BUSY. On initialization failure,
 * call MX_CAN_Init() again before retrying (cleanup may deinitialize CAN).
 */
HAL_StatusTypeDef CanBus_Init(uint8_t motor_id);

/* Consume the latest command. Both output pointers must be non-NULL.
 * The caller is responsible for command timeout and motor control.
 */
uint8_t CanBus_TakeCommand(int16_t *voltage_mv, uint32_t *received_ms);

/* Feedback is encoded as signed little-endian integers; bytes 6 and 7 are zero. */
void CanBus_SetFeedback(int32_t angle_mrad, int16_t speed_rpm_x10);

/* Call regularly from the main loop. Makes at most one TX attempt per 2 ms. */
void CanBus_Task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
