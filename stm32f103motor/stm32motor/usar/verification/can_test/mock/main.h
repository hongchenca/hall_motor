#ifndef CAN_TEST_MAIN_H
#define CAN_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef enum { HAL_CAN_STATE_RESET=0, HAL_CAN_STATE_READY=1,
    HAL_CAN_STATE_LISTENING=2, HAL_CAN_STATE_ERROR=3 } HAL_CAN_StateTypeDef;
typedef struct { uint32_t ignored; } CAN_TypeDef;
typedef struct { CAN_TypeDef *Instance; HAL_CAN_StateTypeDef State; } CAN_HandleTypeDef;
typedef struct {
    uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, FilterBank, FilterMode, FilterScale;
    uint32_t FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp, FilterMatchIndex; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
#define CAN1 ((CAN_TypeDef *)0x40006400U)
#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_FILTER_FIFO0 0U
#define CAN_RX_FIFO0 0U
#define CAN_FILTER_ENABLE 1U
#define CAN_FILTER_DISABLE 0U
#define CAN_IT_RX_FIFO0_MSG_PENDING 2U
#define DISABLE 0U
uint32_t Mock_GetPrimask(void);
void Mock_DisableIrq(void);
void Mock_SetPrimask(uint32_t value);
#define __get_PRIMASK Mock_GetPrimask
#define __disable_irq Mock_DisableIrq
#define __set_PRIMASK Mock_SetPrimask
uint32_t HAL_GetTick(void);
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t mask);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *h, uint32_t mask);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox);
#endif
