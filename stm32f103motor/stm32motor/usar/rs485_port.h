#ifndef RS485_PORT_H
#define RS485_PORT_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Raw-byte transport on USART1 remapped to PB6 TX / PB7 RX.
 * PB6 connects to SP485 DI; PB7 connects to RO. PB5 drives tied DE and /RE:
 * high = transmit, low = receive. USART1 pin remapping is configured by CubeMX.
 * Call RS485_Init() after MX_GPIO_Init() and MX_USART1_UART_Init().
 * Serialize calls and reserve huart1 for this port; do not access it directly.
 * Reception is buffered by the USART1 RX interrupt;
 * Modbus framing is handled by the application layer. Transmit is blocking,
 * while Receive reads the interrupt-fed ring buffer. A zero length is a no-op.
 */
HAL_StatusTypeDef RS485_Init(void);
HAL_StatusTypeDef RS485_Transmit(const uint8_t *data, uint16_t length,
                               uint32_t timeout_ms);
HAL_StatusTypeDef RS485_Receive(uint8_t *data, uint16_t length,
                              uint32_t timeout_ms);
uint32_t RS485_GetRxOverrunCount(void);

#ifdef __cplusplus
}
#endif

#endif
