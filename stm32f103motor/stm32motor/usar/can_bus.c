#include "can_bus.h"
#include "can.h"

#define CAN_BUS_FEEDBACK_PERIOD_MS 2U

static volatile uint8_t bus_started;
static uint8_t bus_motor_id;
static uint8_t bus_command_offset;
static uint32_t bus_command_id;
static uint32_t bus_last_tx_ms;

static volatile uint8_t command_pending;
static volatile int16_t command_voltage_mv;
static volatile uint32_t command_received_ms;
static volatile int32_t feedback_angle_mrad;
static volatile int16_t feedback_speed_rpm_x10;

/* Disable this module's resources. A failed HAL start can leave ERROR state,
 * in which Stop/ConfigFilter cannot operate; DeInit handles that case.
 */
static void CanBus_Cleanup(CAN_FilterTypeDef *filter)
{
    HAL_StatusTypeDef cleanup_status;

    bus_started = 0U;
    command_pending = 0U;
    cleanup_status = HAL_CAN_DeactivateNotification(
        &hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (HAL_CAN_GetState(&hcan) == HAL_CAN_STATE_LISTENING)
    {
        if (HAL_CAN_Stop(&hcan) != HAL_OK)
        {
            cleanup_status = HAL_ERROR;
        }
    }

    filter->FilterActivation = CAN_FILTER_DISABLE;
    if (HAL_CAN_ConfigFilter(&hcan, filter) != HAL_OK)
    {
        cleanup_status = HAL_ERROR;
    }
    if ((cleanup_status != HAL_OK) ||
        (HAL_CAN_GetState(&hcan) == HAL_CAN_STATE_ERROR))
    {
        (void)HAL_CAN_DeInit(&hcan);
    }
}

HAL_StatusTypeDef CanBus_Init(uint8_t motor_id)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    uint32_t primask;

    if ((motor_id < 1U) || (motor_id > 8U) || (hcan.Instance != CAN1))
    {
        return HAL_ERROR;
    }
    if (bus_started != 0U)
    {
        return HAL_BUSY;
    }
    if (HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY)
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    bus_motor_id = motor_id;
    bus_command_id = (motor_id <= 4U) ? 0x100U : 0x200U;
    bus_command_offset = (uint8_t)(((motor_id - 1U) % 4U) * 2U);
    command_pending = 0U;
    command_voltage_mv = 0;
    command_received_ms = 0U;
    feedback_angle_mrad = 0;
    feedback_speed_rpm_x10 = 0;
    bus_last_tx_ms = HAL_GetTick();
    __set_PRIMASK(primask);

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = bus_command_id << 5;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0x7FFU << 5;
    /* In a 32-bit bxCAN filter, bit 2 is IDE and bit 1 is RTR. */
    filter.FilterMaskIdLow = 0x0006U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14U;

    status = HAL_CAN_ConfigFilter(&hcan, &filter);
    if (status != HAL_OK)
    {
        CanBus_Cleanup(&filter);
        return status;
    }
    status = HAL_CAN_Start(&hcan);
    if (status != HAL_OK)
    {
        CanBus_Cleanup(&filter);
        return status;
    }

    /* Publish initialized state before enabling a possibly pending RX IRQ. */
    bus_started = 1U;
    status = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK)
    {
        CanBus_Cleanup(&filter);
        return status;
    }
    return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
    if ((can_handle == NULL) || (can_handle->Instance != CAN1))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(can_handle, CAN_RX_FIFO0) != 0U)
    {
        CAN_RxHeaderTypeDef header = {0};
        uint8_t data[8] = {0};
        uint16_t raw_voltage;
        int32_t signed_voltage;

        if (HAL_CAN_GetRxMessage(can_handle, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        if ((bus_started == 0U) || (header.IDE != CAN_ID_STD) ||
            (header.RTR != CAN_RTR_DATA) || (header.DLC != 8U) ||
            (header.StdId != bus_command_id))
        {
            continue;
        }

        raw_voltage = (uint16_t)((uint16_t)data[bus_command_offset] |
            ((uint16_t)data[bus_command_offset + 1U] << 8));
        signed_voltage = (int32_t)raw_voltage;
        if ((raw_voltage & 0x8000U) != 0U)
        {
            signed_voltage -= 65536;
        }
        command_voltage_mv = (int16_t)signed_voltage;
        command_received_ms = HAL_GetTick();
        command_pending = 1U;
    }
}

uint8_t CanBus_TakeCommand(int16_t *voltage_mv, uint32_t *received_ms)
{
    uint8_t pending;
    uint32_t primask;

    if ((voltage_mv == NULL) || (received_ms == NULL))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    pending = command_pending;
    if (pending != 0U)
    {
        *voltage_mv = command_voltage_mv;
        *received_ms = command_received_ms;
        command_pending = 0U;
    }
    __set_PRIMASK(primask);
    return pending;
}

void CanBus_SetFeedback(int32_t angle_mrad, int16_t speed_rpm_x10)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    feedback_angle_mrad = angle_mrad;
    feedback_speed_rpm_x10 = speed_rpm_x10;
    __set_PRIMASK(primask);
}

void CanBus_Task(uint32_t now_ms)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t mailbox = 0U;
    uint32_t angle_bits;
    uint16_t speed_bits;
    uint32_t primask;

    if ((bus_started == 0U) ||
        ((uint32_t)(now_ms - bus_last_tx_ms) < CAN_BUS_FEEDBACK_PERIOD_MS))
    {
        return;
    }
    bus_last_tx_ms = now_ms;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    angle_bits = (uint32_t)feedback_angle_mrad;
    speed_bits = (uint16_t)feedback_speed_rpm_x10;
    __set_PRIMASK(primask);

    header.StdId = 0x100U + bus_motor_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    data[0] = (uint8_t)angle_bits;
    data[1] = (uint8_t)(angle_bits >> 8);
    data[2] = (uint8_t)(angle_bits >> 16);
    data[3] = (uint8_t)(angle_bits >> 24);
    data[4] = (uint8_t)speed_bits;
    data[5] = (uint8_t)(speed_bits >> 8);

    /* HAL queues one frame; mailbox contention is retried at the next period. */
    (void)HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}
