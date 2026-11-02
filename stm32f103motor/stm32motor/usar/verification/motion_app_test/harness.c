/* Mock peripherals only; production UART parser and motion controller linked intact. */
#include "motor_app.h"
#include "motor_motion.h"
#include "motor_config.h"
#include "bldc_motor.h"
#include "can_bus.h"
#include "actuators.h"
#include "rs485_port.h"
#include <string.h>

TIM_TypeDef mock_tim1, mock_tim3, mock_tim4;
GPIO_TypeDef mock_gpioa, mock_gpiob;
AFIO_TypeDef mock_afio;
RCC_TypeDef mock_rcc;
USART_TypeDef mock_usart3;
UART_HandleTypeDef huart3;
static BldcStatus driver;
static Mt6v10Status encoder;
static uint32_t now_ms, primask, rx_pending, rx_byte;
static int16_t can_voltage;
static uint32_t can_timestamp, can_pending;
int32_t requested_voltage, requested_period, emergency_count, command_timestamp;
int32_t dc_duty, servo_pulse;
uint32_t blocking_transmit_calls;
char test_command[256];
int32_t test_snapshot[20];

uint32_t Mock_GetPrimask(void) { return primask; }
void Mock_SetPrimask(uint32_t mask) { primask = mask; }
uint32_t HAL_GetTick(void) { return now_ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint16_t pin, GPIO_PinState state)
{ (void)gpio; (void)pin; (void)state; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
    (void)uart; (void)timeout; (void)data; (void)length;
    blocking_transmit_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t length, uint32_t timeout)
{
    (void)uart; (void)length; (void)timeout;
    if (!rx_pending) { return HAL_TIMEOUT; }
    *data = (uint8_t)rx_byte; rx_pending = 0;
    return HAL_OK;
}
HAL_StatusTypeDef Bldc_Init(void)
{
    memset(&driver, 0, sizeof(driver));
    driver.vbus_mv = 24000; driver.vbus_adc = 2708;
    driver.voltage_ready = 1; driver.open_loop_test = MOTOR_OPEN_LOOP_TEST;
    requested_period = MOTOR_OPEN_LOOP_COMMUTATION_TICKS;
    requested_voltage = 0; emergency_count = 0;
    return HAL_OK;
}
void Bldc_SetVoltageMv(int16_t mv, uint32_t received_ms)
{
    requested_voltage = mv; command_timestamp = (int32_t)received_ms;
    driver.state = mv ? BLDC_RUNNING : BLDC_STOPPED;
    driver.duty_ticks = mv ? 320 : 0;
}
void Bldc_SetCommutationTicks(uint16_t ticks)
{ requested_period = ticks; driver.commutation_interval_ticks = ticks; }
void Bldc_EmergencyStop(void)
{
    emergency_count++; requested_voltage = 0;
    requested_period = MOTOR_OPEN_LOOP_COMMUTATION_TICKS;
    driver.state = BLDC_STOPPED; driver.duty_ticks = 0;
}
void Bldc_GetStatus(BldcStatus *status) { *status = driver; }
void Bldc_GetBreakDiagnostic(BldcBreakDiagnostic *status) { memset(status, 0, sizeof(*status)); }
HAL_StatusTypeDef Bldc_ClearFault(void)
{
    if (requested_voltage != 0 || !driver.voltage_ready) { return HAL_ERROR; }
    driver.fault = BLDC_FAULT_NONE; return HAL_OK;
}
void Bldc_FastTick(void) {}
void Bldc_BreakIrq(void) {}
HAL_StatusTypeDef Actuators_Init(void) { dc_duty = 0; servo_pulse = 1500; return HAL_OK; }
void Actuators_Stop(void) { dc_duty = 0; }
void BrushedMotor_SetDuty(int16_t duty) { dc_duty = duty; }
void Servo_SetPulseUs(uint16_t pulse) { servo_pulse = pulse; }
HAL_StatusTypeDef RS485_Init(void) { return HAL_OK; }
uint32_t RS485_GetRxOverrunCount(void) { return 0; }
HAL_StatusTypeDef CanBus_Init(uint8_t id) { (void)id; can_pending = 0; return HAL_OK; }
uint8_t CanBus_TakeCommand(int16_t *mv, uint32_t *ms)
{
    if (!can_pending) { return 0; }
    can_pending = 0; *mv = can_voltage; *ms = can_timestamp; return 1;
}
void CanBus_SetFeedback(int32_t angle, int16_t speed) { (void)angle; (void)speed; }
void CanBus_Task(uint32_t now) { (void)now; }
void Mt6v10_Init(uint32_t now) { (void)now; memset(&encoder, 0, sizeof(encoder)); }
void Mt6v10_Task(uint32_t now) { (void)now; }
void Mt6v10_GetStatus(Mt6v10Status *status) { *status = encoder; }
uint8_t Mt6v10_TakeUpdate(Mt6v10Status *status) { (void)status; return 0; }

void Test_Reset(void)
{
    now_ms = 1000; primask = 0; rx_pending = blocking_transmit_calls = 0;
    mock_usart3.SR = USART_SR_TXE; mock_usart3.DR = 0;
    (void)MotorApp_Init();
}
void Test_Encoder(uint32_t raw, uint32_t sample_ms, uint32_t flags)
{
    encoder.angle_raw = (uint16_t)raw;
    encoder.turn_count = (int16_t)(raw >> 16);
    encoder.sample_ms = sample_ms;
    encoder.has_sample = (uint8_t)(flags & 1);
    encoder.online = (uint8_t)((flags >> 1) & 1);
}
void Test_Run(uint32_t now) { now_ms = now; MotorApp_Process(); }
void Test_Driver(uint32_t ready, uint32_t fault)
{ driver.voltage_ready = (uint8_t)ready; driver.fault = (BldcFault)fault; }
void Test_Can(int32_t mv, uint32_t timestamp)
{ can_voltage = (int16_t)mv; can_timestamp = timestamp; can_pending = 1; }
void Test_Command(void)
{
    uint32_t i;
    for (i = 0; test_command[i]; i++) {
        rx_byte = (uint8_t)test_command[i]; rx_pending = 1;
        MotorApp_Process();
    }
}
void Test_Snapshot(void)
{
    MotorMotionStatus s;
    MotorMotion_GetStatus(&s);
    test_snapshot[0] = s.state; test_snapshot[1] = s.fault;
    test_snapshot[2] = s.active; test_snapshot[3] = s.position_mode;
    test_snapshot[4] = s.feedback_ready; test_snapshot[5] = s.reference_valid;
    test_snapshot[6] = s.direction; test_snapshot[7] = s.drive_direction;
    test_snapshot[8] = s.rpm_limit; test_snapshot[9] = s.interval_ticks;
    test_snapshot[10] = s.speed_rpm_x10; test_snapshot[11] = s.ramp_rpm_x10;
    test_snapshot[12] = s.position_deg_x10; test_snapshot[13] = s.target_deg_x10;
    test_snapshot[14] = requested_voltage; test_snapshot[15] = requested_period;
    test_snapshot[16] = emergency_count; test_snapshot[17] = driver.fault;
    test_snapshot[18] = (int32_t)now_ms; test_snapshot[19] = command_timestamp;
}
void test_entry(void) { Test_Reset(); }
