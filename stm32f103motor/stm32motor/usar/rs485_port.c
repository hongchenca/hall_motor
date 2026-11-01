#include "rs485_port.h"
#include "main.h"
#include "usart.h"

#define RS485_RX_BUFFER_SIZE 64U
#define RS485_RX_BUFFER_MASK (RS485_RX_BUFFER_SIZE - 1U)

static uint8_t rs485_rx_interrupt_byte;
static volatile uint8_t rs485_rx_buffer[RS485_RX_BUFFER_SIZE];
static volatile uint8_t rs485_rx_head;
static volatile uint8_t rs485_rx_tail;
static volatile uint32_t rs485_rx_overruns;
static volatile uint8_t rs485_rx_started;

static void RS485_SetReceive(void)
{
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef RS485_Init(void)
{
    HAL_StatusTypeDef status;

    if (rs485_rx_started != 0U) {
        return HAL_OK;
    }
    RS485_SetReceive();
    rs485_rx_head = 0U;
    rs485_rx_tail = 0U;
    rs485_rx_overruns = 0U;
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    status = HAL_UART_Receive_IT(&huart1, &rs485_rx_interrupt_byte, 1U);
    if (status == HAL_OK) {
        rs485_rx_started = 1U;
    }
    return status;
}

HAL_StatusTypeDef RS485_Transmit(const uint8_t *data, uint16_t length,
                               uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if (length == 0U)
    {
        RS485_SetReceive();
        return HAL_OK;
    }
    if (data == NULL)
    {
        RS485_SetReceive();
        return HAL_ERROR;
    }
    if (rs485_rx_started == 0U) {
        RS485_SetReceive();
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
    /* The project's blocking HAL_UART_Transmit waits for UART_FLAG_TC,
     * including the final stop bit, before returning HAL_OK. A timeout or
     * other error also releases the bus by returning DIR to receive mode.
     */
    status = HAL_UART_Transmit(&huart1, data, length, timeout_ms);
    RS485_SetReceive();
    return status;
}

HAL_StatusTypeDef RS485_Receive(uint8_t *data, uint16_t length,
                              uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();
    uint16_t index;

    RS485_SetReceive();
    if (length == 0U)
    {
        return HAL_OK;
    }
    if (data == NULL)
    {
        return HAL_ERROR;
    }
    if (rs485_rx_started == 0U) {
        return HAL_ERROR;
    }

    for (index = 0U; index < length; ++index) {
        uint8_t tail;
        while (rs485_rx_tail == rs485_rx_head) {
            if (timeout_ms == 0U ||
                (timeout_ms != HAL_MAX_DELAY &&
                 (uint32_t)(HAL_GetTick() - tickstart) > timeout_ms)) {
                return HAL_TIMEOUT;
            }
        }
        tail = rs485_rx_tail;
        data[index] = rs485_rx_buffer[tail];
        rs485_rx_tail = (uint8_t)((tail + 1U) & RS485_RX_BUFFER_MASK);
    }
    return HAL_OK;
}

uint32_t RS485_GetRxOverrunCount(void)
{
    return rs485_rx_overruns;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->Instance != USART1) {
        return;
    }

    {
        uint8_t head = rs485_rx_head;
        uint8_t next = (uint8_t)((head + 1U) & RS485_RX_BUFFER_MASK);
        if (next != rs485_rx_tail) {
            rs485_rx_buffer[head] = rs485_rx_interrupt_byte;
            rs485_rx_head = next;
        } else {
            rs485_rx_overruns++;
        }
    }
    rs485_rx_started =
        (HAL_UART_Receive_IT(uart, &rs485_rx_interrupt_byte, 1U) == HAL_OK) ? 1U : 0U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->Instance != USART1) {
        return;
    }

    if ((uart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) {
        rs485_rx_overruns++;
    }
    __HAL_UART_CLEAR_PEFLAG(uart);
    if (uart->RxState == HAL_UART_STATE_READY) {
        rs485_rx_started =
            (HAL_UART_Receive_IT(uart, &rs485_rx_interrupt_byte, 1U) == HAL_OK) ? 1U : 0U;
    }
}
