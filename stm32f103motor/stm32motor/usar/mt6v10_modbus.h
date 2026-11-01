#ifndef MT6V10_MODBUS_H
#define MT6V10_MODBUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t online;
    uint8_t has_sample;
    uint16_t angle_raw;
    int16_t turn_count;
    int32_t angle_mrad;
    int16_t speed_rpm_x10;
    uint32_t sample_ms;
    uint32_t crc_errors;
    uint32_t response_timeouts;
    uint32_t protocol_errors;
    uint32_t tx_errors;
} Mt6v10Status;

/* Polls angle register 1 and signed multi-turn register 2 at 38400 8N1.
 * Call from foreground code; response reception is byte-polled without waits.
 */
void Mt6v10_Init(uint32_t now_ms);
void Mt6v10_Task(uint32_t now_ms);
void Mt6v10_GetStatus(Mt6v10Status *status);
uint8_t Mt6v10_TakeUpdate(Mt6v10Status *status);

#ifdef __cplusplus
}
#endif

#endif
