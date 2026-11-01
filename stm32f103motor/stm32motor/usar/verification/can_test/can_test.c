/* Executed as ARM machine code; can_bus.c is compiled unmodified separately. */
#include "can_bus.h"
#include "can.h"

CAN_HandleTypeDef hcan = { CAN1, HAL_CAN_STATE_READY };
volatile uint32_t test_checks, test_failed_line, test_done;
static uint32_t tick, primask, disable_calls, restore_calls;
static uint32_t config_calls, start_calls, activate_calls, deactivate_calls;
static uint32_t stop_calls, deinit_calls, tx_calls, rx_calls;
static uint32_t sequence, config_order, start_order, activate_order;
static uint32_t fail_config, fail_start, fail_activate, fail_deactivate, fail_stop;
static uint32_t inject_pending_irq;
static uint32_t rx_fail, free_mailboxes = 3U;
static HAL_StatusTypeDef tx_status = HAL_OK;
static CAN_FilterTypeDef first_filter, last_filter;
static CAN_TxHeaderTypeDef last_tx;
static uint8_t last_data[8];
typedef struct { CAN_RxHeaderTypeDef h; uint8_t data[8]; } Frame;
static Frame queue[20];
static uint32_t qread, qwrite;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h);

#define CHECK(x) do { ++test_checks; if (!(x)) { test_failed_line=__LINE__; return; } } while (0)

/* Compiler-generated zero/copy operations, with no C library dependency. */
void __aeabi_memclr(void *p, unsigned n) { volatile uint8_t *v=p; while(n--) *v++=0; }
void __aeabi_memclr4(void *p, unsigned n) { __aeabi_memclr(p,n); }
void __aeabi_memcpy(void *d, const void *s, unsigned n) {
    volatile uint8_t *dst=d; const volatile uint8_t *src=s;
    while(n--) *dst++=*src++;
}
void __aeabi_memcpy4(void *d, const void *s, unsigned n) { __aeabi_memcpy(d,s,n); }
void __aeabi_memcpy8(void *d, const void *s, unsigned n) { __aeabi_memcpy(d,s,n); }
uint32_t Mock_GetPrimask(void) { return primask; }
void Mock_DisableIrq(void) { primask=1U; ++disable_calls; }
void Mock_SetPrimask(uint32_t value) { primask=value; ++restore_calls; }
uint32_t HAL_GetTick(void) { return tick; }
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *h) { return h->State; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f) {
    ++config_calls; ++sequence; last_filter=*f;
    if(config_calls==1U) { first_filter=*f; config_order=sequence; }
    if((fail_config && config_calls==1U) || h->State==HAL_CAN_STATE_ERROR) return HAL_ERROR;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) {
    ++start_calls; start_order=++sequence;
    if(fail_start) { h->State=HAL_CAN_STATE_ERROR; return HAL_TIMEOUT; }
    h->State=HAL_CAN_STATE_LISTENING; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h) {
    ++stop_calls;
    if(fail_stop) { h->State=HAL_CAN_STATE_ERROR; return HAL_TIMEOUT; }
    h->State=HAL_CAN_STATE_READY; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *h) {
    ++deinit_calls; h->State=HAL_CAN_STATE_RESET; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t mask) {
    (void)mask; ++activate_calls; activate_order=++sequence;
    if(inject_pending_irq) HAL_CAN_RxFifo0MsgPendingCallback(h);
    return fail_activate ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *h, uint32_t mask) {
    (void)mask; ++deactivate_calls;
    return (fail_deactivate || h->State==HAL_CAN_STATE_ERROR) ? HAL_ERROR : HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo) {
    (void)h; (void)fifo; return qwrite-qread;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data) {
    uint32_t i; (void)h; (void)fifo; ++rx_calls;
    if(rx_fail) return HAL_ERROR;
    *header=queue[qread].h;
    for(i=0;i<8U;++i) data[i]=queue[qread].data[i];
    ++qread; return HAL_OK;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h) { (void)h; return free_mailboxes; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox) {
    uint32_t i; (void)h; ++tx_calls; last_tx=*header;
    for(i=0;i<8U;++i) last_data[i]=data[i];
    *mailbox=1; return tx_status;
}
static void Push(uint32_t id, uint32_t ide, uint32_t rtr, uint32_t dlc, const uint8_t *data) {
    uint32_t i; Frame *f=&queue[qwrite++];
    f->h.StdId=id; f->h.ExtId=id; f->h.IDE=ide; f->h.RTR=rtr; f->h.DLC=dlc;
    for(i=0;i<8U;++i) f->data[i]=data[i];
}
static uint32_t FilterAccepts(uint32_t packed) {
    uint32_t value=(first_filter.FilterIdHigh<<16)|first_filter.FilterIdLow;
    uint32_t mask=(first_filter.FilterMaskIdHigh<<16)|first_filter.FilterMaskIdLow;
    return ((value^packed)&mask)==0U;
}
static void Success(uint32_t motor) {
    static const uint8_t slots[8]={0x34,0x12,0xfe,0xff,0,0x80,0xff,0x7f};
    static const int16_t expected[4]={4660,-2,-32768,32767};
    static const uint8_t newer[8]={0xfd,2,0xfd,2,0xfd,2,0xfd,2};
    static const uint8_t fb[8]={0xc0,0x1d,0xfe,0xff,0x2e,0xfb,0,0};
    uint32_t id=motor<=4U?0x100U:0x200U, stamp=999U, before, i;
    int16_t voltage=999;
    CAN_HandleTypeDef foreign={ (CAN_TypeDef *)0x40006800U, HAL_CAN_STATE_LISTENING };
    CHECK(CanBus_Init(0U)==HAL_ERROR);
    CHECK(CanBus_Init(9U)==HAL_ERROR);
    CHECK(config_calls==0U);
    tick=100U; primask=1U;
    CHECK(CanBus_Init((uint8_t)motor)==HAL_OK);
    CHECK(primask==1U && disable_calls==restore_calls);
    CHECK(config_calls==1U && start_calls==1U && activate_calls==1U);
    CHECK(config_order<start_order && start_order<activate_order);
    CHECK(first_filter.FilterBank==0U && first_filter.FilterFIFOAssignment==CAN_FILTER_FIFO0);
    CHECK(first_filter.FilterMode==CAN_FILTERMODE_IDMASK && first_filter.FilterScale==CAN_FILTERSCALE_32BIT);
    CHECK(first_filter.FilterActivation==CAN_FILTER_ENABLE && first_filter.SlaveStartFilterBank==14U);
    CHECK(FilterAccepts(id<<21));
    CHECK(!FilterAccepts((id+1U)<<21));
    CHECK(!FilterAccepts((id<<21)|CAN_ID_EXT));
    CHECK(!FilterAccepts((id<<21)|CAN_RTR_REMOTE));
    CHECK(CanBus_Init((uint8_t)motor)==HAL_BUSY);
    CHECK(CanBus_TakeCommand(&voltage,&stamp)==0U && voltage==999 && stamp==999U);

    tick=101U;
    Push(id,CAN_ID_STD,CAN_RTR_DATA,8U,slots);
    HAL_CAN_RxFifo0MsgPendingCallback(NULL);
    HAL_CAN_RxFifo0MsgPendingCallback(&foreign);
    CHECK(qread==0U && rx_calls==0U);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(qread==qwrite && rx_calls==1U);
    CHECK(CanBus_TakeCommand(NULL,&stamp)==0U);
    CHECK(CanBus_TakeCommand(&voltage,NULL)==0U);
    CHECK(CanBus_TakeCommand(&voltage,&stamp)==1U);
    CHECK(voltage==expected[(motor-1U)%4U] && stamp==101U);
    CHECK(primask==1U && disable_calls==restore_calls);
    CHECK(CanBus_TakeCommand(&voltage,&stamp)==0U);

    Push(id+1U,CAN_ID_STD,CAN_RTR_DATA,8U,newer);
    Push(id,CAN_ID_EXT,CAN_RTR_DATA,8U,newer);
    Push(id,CAN_ID_STD,CAN_RTR_REMOTE,8U,newer);
    Push(id,CAN_ID_STD,CAN_RTR_DATA,7U,newer);
    Push(id,CAN_ID_STD,CAN_RTR_DATA,9U,newer);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(qread==qwrite && rx_calls==6U);
    CHECK(CanBus_TakeCommand(&voltage,&stamp)==0U);

    tick=104U;
    Push(id,CAN_ID_STD,CAN_RTR_DATA,8U,slots);
    Push(id,CAN_ID_STD,CAN_RTR_DATA,8U,newer);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(qread==qwrite);
    CHECK(CanBus_TakeCommand(&voltage,&stamp)==1U && voltage==765 && stamp==104U);
    primask=0U;
    CanBus_SetFeedback(-123456,-1234);
    CHECK(primask==0U && disable_calls==restore_calls);
    CanBus_Task(100U); CanBus_Task(101U);
    CHECK(tx_calls==0U);
    CanBus_Task(102U);
    CHECK(tx_calls==1U && last_tx.StdId==0x100U+motor);
    CHECK(last_tx.IDE==CAN_ID_STD && last_tx.RTR==CAN_RTR_DATA && last_tx.DLC==8U);
    CHECK(last_tx.ExtId==0U && last_tx.TransmitGlobalTime==DISABLE);
    for(i=0;i<8U;++i) CHECK(last_data[i]==fb[i]);
    CanBus_Task(102U); CanBus_Task(103U);
    CHECK(tx_calls==1U);
    CanBus_Task(104U); CHECK(tx_calls==2U);
    free_mailboxes=0U; CanBus_Task(106U); CHECK(tx_calls==2U);
    free_mailboxes=3U; CanBus_Task(107U); CHECK(tx_calls==2U);
    CanBus_Task(108U); CHECK(tx_calls==3U);
    tx_status=HAL_ERROR; CanBus_Task(110U); CHECK(tx_calls==4U);
    tx_status=HAL_OK; CanBus_Task(112U); CHECK(tx_calls==5U);

    /* Timeout belongs to the application: the module retains old commands. */
    tick=120U;
    Push(id,CAN_ID_STD,CAN_RTR_DATA,8U,slots);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CanBus_Task(621U);
    CHECK(CanBus_TakeCommand(&voltage,&stamp)==1U && stamp==120U);
    CHECK(voltage==expected[(motor-1U)%4U]);
    before=rx_calls; rx_fail=1U;
    Push(id,CAN_ID_STD,CAN_RTR_DATA,8U,slots);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(rx_calls==before+1U && qread<qwrite);
    CHECK(CanBus_TakeCommand(&voltage,&stamp)==0U);
    rx_fail=0U; HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(qread==qwrite && CanBus_TakeCommand(&voltage,&stamp)==1U);
    CHECK(primask==0U && disable_calls==restore_calls);
}
static void Failure(uint32_t mode) {
    int16_t v=9; uint32_t t=9;
    if(mode==1U) fail_config=1U;
    if(mode==2U) fail_start=1U;
    if(mode>=3U) fail_activate=1U;
    if(mode==4U) fail_deactivate=1U;
    if(mode==5U) fail_stop=1U;
    CHECK(CanBus_Init(1U)==(mode==2U?HAL_TIMEOUT:HAL_ERROR));
    CHECK(config_calls==2U && deactivate_calls==1U);
    CHECK(last_filter.FilterActivation==CAN_FILTER_DISABLE);
    CHECK(start_calls==(mode==1U?0U:1U));
    CHECK(activate_calls==(mode<=2U?0U:1U));
    CHECK(stop_calls==(mode>=3U?1U:0U));
    CHECK(deinit_calls==((mode==2U||mode==4U||mode==5U)?1U:0U));
    CanBus_Task(1000U); CHECK(tx_calls==0U);
    CHECK(CanBus_TakeCommand(&v,&t)==0U && v==9 && t==9U);
    /* Simulates rerunning MX_CAN_Init after documented cleanup. */
    hcan.State=HAL_CAN_STATE_READY;
    fail_config=fail_start=fail_activate=fail_deactivate=fail_stop=0U;
    CHECK(CanBus_Init(1U)==HAL_OK);
    CanBus_Task(2U); CHECK(tx_calls==1U);
}
static void InvalidState(void) {
    hcan.Instance=(CAN_TypeDef *)0x40006800U;
    CHECK(CanBus_Init(1U)==HAL_ERROR);
    hcan.Instance=CAN1; hcan.State=HAL_CAN_STATE_LISTENING;
    CHECK(CanBus_Init(1U)==HAL_ERROR);
    hcan.State=HAL_CAN_STATE_RESET;
    CHECK(CanBus_Init(1U)==HAL_ERROR);
    CHECK(config_calls==0U && start_calls==0U && activate_calls==0U);
}
static void TickWrap(void) {
    tick=0xfffffffeU;
    CHECK(CanBus_Init(8U)==HAL_OK);
    CanBus_Task(0xffffffffU); CHECK(tx_calls==0U);
    CanBus_Task(0U); CHECK(tx_calls==1U);
    CanBus_Task(1U); CHECK(tx_calls==1U);
    CanBus_Task(2U); CHECK(tx_calls==2U);
}
static void PendingIRQ(uint32_t failing) {
    static const uint8_t data[8]={0x00,0x80,0,0,0,0,0,0};
    int16_t v=7; uint32_t t=7;
    Push(0x100U,CAN_ID_STD,CAN_RTR_DATA,8U,data);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    CHECK(qread==qwrite);
    CHECK(CanBus_TakeCommand(&v,&t)==0U && v==7 && t==7U);
    Push(0x100U,CAN_ID_STD,CAN_RTR_DATA,8U,data);
    inject_pending_irq=1U; fail_activate=failing; tick=77U;
    CHECK(CanBus_Init(1U)==(failing?HAL_ERROR:HAL_OK));
    CHECK(qread==qwrite && rx_calls==2U);
    if(failing) {
        CHECK(CanBus_TakeCommand(&v,&t)==0U && v==7 && t==7U);
        CanBus_Task(100U); CHECK(tx_calls==0U);
    } else {
        CHECK(CanBus_TakeCommand(&v,&t)==1U && v==-32768 && t==77U);
    }
}
void Test_Run(uint32_t mode, uint32_t motor) {
    if(mode==0U) Success(motor);
    else if(mode<=5U) Failure(mode);
    else if(mode==6U) InvalidState();
    else if(mode==7U) TickWrap();
    else if(mode==8U || mode==9U) PendingIRQ(mode==9U);
    else test_failed_line=0xffffffffU;
    test_done=1U;
}
