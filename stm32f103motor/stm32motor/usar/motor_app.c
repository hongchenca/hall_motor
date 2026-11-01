#include "motor_app.h"
#include "motor_config.h"
#include "bldc_motor.h"
#include "can_bus.h"
#include "actuators.h"
#include "mt6v10_modbus.h"
#include "rs485_port.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define CONSOLE_LINE_CAPACITY 64U
#define CONSOLE_BAUD_RATE 115200U
#define CONSOLE_BITS_PER_BYTE 10U
#define CONSOLE_TX_MARGIN_MS 10U
#if MOTOR_OPEN_LOOP_TEST
#define UART_MOTOR_MAX_PERCENT MOTOR_OPEN_LOOP_MAX_DUTY_PERCENT
#else
#define UART_MOTOR_MAX_PERCENT 100U
#endif
#define UART_MOTOR_DEFAULT_PERCENT 10U

static uint8_t app_ready;
static uint32_t led_changed_ms;
static uint8_t led_fault;
static uint8_t led_edge;
static char console_line[CONSOLE_LINE_CAPACITY];
static uint8_t console_length;
static uint8_t console_overflow;
static uint8_t uart_control_active;
static uint8_t uart_motor_running;
static uint8_t uart_motor_speed_percent;
static int8_t uart_motor_direction;
static uint8_t brushed_motor_running;
static uint8_t brushed_motor_speed_percent;
static int8_t brushed_motor_direction;
static uint32_t last_hall_print_ms;
static uint8_t hall_print_active;

static HAL_StatusTypeDef console_transmit(const uint8_t *data, uint16_t length)
{
    uint32_t wire_time_ms =
        (((uint32_t)length * CONSOLE_BITS_PER_BYTE * 1000U) +
         CONSOLE_BAUD_RATE - 1U) / CONSOLE_BAUD_RATE;

    return HAL_UART_Transmit(&huart3, (uint8_t *)data, length,
                             wire_time_ms + CONSOLE_TX_MARGIN_MS);
}

static uint8_t read_hall_gpio(uint32_t *port_a, uint32_t *port_b)
{
    uint8_t pa6;
    uint8_t pa7;
    uint8_t pb0;
    *port_a = GPIOA->IDR;
    *port_b = GPIOB->IDR;
    pa6 = (uint8_t)((*port_a >> 6U) & 1U);
    pa7 = (uint8_t)((*port_a >> 7U) & 1U);
    pb0 = (uint8_t)(*port_b & GPIO_PIN_0);
    return (uint8_t)((pb0 << 2U) | (pa7 << 1U) | pa6);
}

static void console_write(const char *text)
{
    size_t length = strlen(text);
    if (length <= 255U) {
        (void)console_transmit((const uint8_t *)text, (uint16_t)length);
    }
}

static void brushed_motor_apply(void)
{
    int16_t duty_per_mille;
    if (brushed_motor_running == 0U || brushed_motor_speed_percent == 0U) {
        Actuators_Stop();
        return;
    }
    duty_per_mille = (int16_t)((uint16_t)brushed_motor_speed_percent * 10U);
    if (brushed_motor_direction < 0) {
        duty_per_mille = (int16_t)-duty_per_mille;
    }
    BrushedMotor_SetDuty(duty_per_mille);
}

static char *console_next_token(char **cursor)
{
    char *token;
    while (**cursor == ' ' || **cursor == '\t') {
        (*cursor)++;
    }
    if (**cursor == '\0') {
        return NULL;
    }
    token = *cursor;
    while (**cursor != '\0' && **cursor != ' ' && **cursor != '\t') {
        (*cursor)++;
    }
    if (**cursor != '\0') {
        **cursor = '\0';
        (*cursor)++;
    }
    return token;
}

static uint8_t console_parse_uint(const char *text, uint16_t maximum,
                                  uint16_t *value)
{
    uint16_t result = 0U;
    if ((*text < '0') || (*text > '9')) {
        return 0U;
    }
    do {
        uint8_t digit = (uint8_t)(*text - '0');
        if (digit > maximum || result > (uint16_t)((maximum - digit) / 10U)) {
            return 0U;
        }
        result = (uint16_t)(result * 10U + digit);
        text++;
    } while (*text >= '0' && *text <= '9');
    if (*text != '\0' || result > maximum) {
        return 0U;
    }
    *value = result;
    return 1U;
}

static uint8_t console_parse_servo_angle(const char *text, int16_t *angle)
{
    uint8_t negative = 0U;
    uint16_t magnitude;
    if (*text == '-') {
        negative = 1U;
        text++;
    } else if (*text == '+') {
        text++;
    }
    if (!console_parse_uint(text, 60U, &magnitude)) {
        return 0U;
    }
    *angle = (negative != 0U) ? -(int16_t)magnitude : (int16_t)magnitude;
    return 1U;
}

static uint8_t console_has_extra(char **cursor)
{
    return (uint8_t)(console_next_token(cursor) != NULL);
}

static void select_uart_control(void)
{
    if (uart_control_active == 0U) {
        Bldc_EmergencyStop();
        uart_motor_running = 0U;
        uart_control_active = 1U;
    }
}

static const char *break_source_name(BldcBreakSource source)
{
    switch (source) {
    case BLDC_BREAK_INIT_PIN_LOW: return "INIT_LOW";
    case BLDC_BREAK_TICK_PIN_LOW: return "TICK_LOW";
    case BLDC_BREAK_IRQ: return "IRQ";
    case BLDC_BREAK_ENABLE_PRECHECK: return "START_GUARD";
    case BLDC_BREAK_ENABLE_POSTCHECK: return "START_VERIFY";
    default: return "NONE";
    }
}

static void debug_bkin(void)
{
    BldcBreakDiagnostic last;
    uint32_t key = __get_PRIMASK();
    uint32_t bdtr;
    uint32_t sr;
    uint32_t dier;
    uint32_t pb12;
    uint32_t remap;
    uint32_t css;
    char text[256];
    int length;
    __disable_irq();
    Bldc_GetBreakDiagnostic(&last);
    bdtr = TIM1->BDTR;
    sr = TIM1->SR;
    dier = TIM1->DIER;
    pb12 = (GPIOB->IDR & GPIO_PIN_12) != 0U;
    remap = (AFIO->MAPR >> 6U) & 3U;
    css = (RCC->CR & RCC_CR_CSSON) != 0U;
    __set_PRIMASK(key);
    length = snprintf(text, sizeof(text),
        "BKIN live: pb12=%u bke=%u bkp=%u moe=%u bif=%u bie=%u map=%u css=%u\r\n"
        "BKIN last: src=%s pb12=%u state=%u hall=%u sr=%04lX bdtr=%04lX dier=%04lX\r\n",
        (unsigned)pb12, (unsigned)((bdtr & TIM_BDTR_BKE) != 0U),
        (unsigned)((bdtr & TIM_BDTR_BKP) != 0U),
        (unsigned)((bdtr & TIM_BDTR_MOE) != 0U),
        (unsigned)((sr & TIM_SR_BIF) != 0U),
        (unsigned)((dier & TIM_DIER_BIE) != 0U), (unsigned)remap, (unsigned)css,
        break_source_name(last.source), (unsigned)last.pb12,
        (unsigned)last.state, (unsigned)last.hall,
        (unsigned long)last.sr, (unsigned long)last.bdtr, (unsigned long)last.dier);
    if (length > 0 && length < (int)sizeof(text)) {
        (void)console_transmit((const uint8_t *)text, (uint16_t)length);
    }
}

static void debug_status(void)
{
    BldcStatus status;
    Mt6v10Status encoder;
    char text[384];
    int length;
    uint32_t gpioa;
    uint32_t gpiob;
    uint8_t hall_raw = read_hall_gpio(&gpioa, &gpiob);
    Bldc_GetStatus(&status);
    Mt6v10_GetStatus(&encoder);
    length = snprintf(text, sizeof(text),
        "control=%s uart_target=%s direction=%s duty_request=%u%% state=%u fault=%u "
        "vbus=%lu mV adc=%u pa2=%u duty=%u/3200 hall=%u hall_raw=%u "
        "(PB0:PA7:PA6=%u%u%u) timeout=%u dc=%s dir=%s speed=%u%% pwm=%u/3200 "
        "enc=%s angle=%ldmrad rpm_x10=%d raw=%u turns=%d crc=%lu timeout=%lu rx_ovr=%lu\r\n",
        (uart_control_active != 0U) ? "UART" : "CAN",
        (uart_control_active == 0U) ? "N/A" :
            ((uart_motor_running != 0U) ? "RUN" : "STOP"),
        (uart_motor_direction < 0) ? "REV" : "FWD",
        (unsigned)uart_motor_speed_percent,
        (unsigned)status.state, (unsigned)status.fault,
        (unsigned long)status.vbus_mv, (unsigned)status.vbus_adc,
        (unsigned)status.pa2_adc, (unsigned)status.duty_ticks,
        (unsigned)status.hall, (unsigned)hall_raw,
        (unsigned)((gpiob & GPIO_PIN_0) != 0U),
        (unsigned)((gpioa & GPIO_PIN_7) != 0U),
        (unsigned)((gpioa & GPIO_PIN_6) != 0U),
        (unsigned)status.command_timed_out,
        (brushed_motor_running != 0U) ? "RUN" : "STOP",
        (brushed_motor_direction < 0) ? "REV" : "FWD",
        (unsigned)brushed_motor_speed_percent,
        (unsigned)((brushed_motor_running != 0U)
            ? ((uint16_t)brushed_motor_speed_percent * 32U) : 0U),
        (encoder.online != 0U) ? "online" : "offline",
        (long)encoder.angle_mrad, (int)encoder.speed_rpm_x10,
        (unsigned)encoder.angle_raw, (int)encoder.turn_count,
        (unsigned long)encoder.crc_errors,
        (unsigned long)encoder.response_timeouts,
        (unsigned long)RS485_GetRxOverrunCount());
    if (length > 0 && length < (int)sizeof(text)) {
        (void)console_transmit((const uint8_t *)text, (uint16_t)length);
    }
    length = snprintf(text, sizeof(text),
        "commutation=%s hall_used=%u step=%u interval_us=%lu\r\n",
        (status.open_loop_test != 0U) ? "OPEN_LOOP_TEST" : "HALL",
        (unsigned)(status.open_loop_test == 0U),
        (unsigned)status.commutation_step,
        (unsigned long)status.commutation_interval_ticks * 50UL);
    if (length > 0 && length < (int)sizeof(text)) {
        (void)console_transmit((const uint8_t *)text, (uint16_t)length);
    }
    debug_bkin();
}

static void console_motor_speed_range(void)
{
    char text[96];
    (void)snprintf(text, sizeof(text),
        "MOTOR START default is %u%% duty; MOTOR SPEED accepts 0 or 8..%u%%.\r\n",
        (unsigned)UART_MOTOR_DEFAULT_PERCENT, (unsigned)UART_MOTOR_MAX_PERCENT);
    console_write(text);
}

static void console_help(void)
{
    console_write("Commands (CR/LF): MOTOR SPEED n | MOTOR START/STOP | "
                  "MOTOR DIR FWD/REV | DC SPEED 0..100 | DC START/STOP | "
                  "DC DIR FWD/REV | SERVO ANGLE -60..60 | MODE CAN/UART | "
                  "STATUS | BKIN | FAULT CLEAR | HELP\r\n");
    console_motor_speed_range();
#if MOTOR_OPEN_LOOP_TEST
    console_write("Six-step open-loop; Hall prints every 100 ms while running, "
                  "commutation interval is 100 ms.\r\n");
#endif
    console_write("DC uses PA4/PA5 direction and PB1 PWM; direction changes stop first.\r\n");
}

static void console_clear_fault(void)
{
    if (Bldc_ClearFault() == HAL_OK) {
        console_write("OK fault cleared\r\n");
    } else {
        console_write("ERR fault clear blocked\r\n");
    }
}

static void console_process_line(void)
{
    char *cursor = console_line;
    char *command = console_next_token(&cursor);
    char *argument;
    uint16_t value;

    if (command == NULL) {
        return;
    }
    if (strcmp(command, "HELP") == 0) {
        if (!console_has_extra(&cursor)) {
            console_help();
        } else {
            console_write("ERR usage: HELP\r\n");
        }
        return;
    }
    if (strcmp(command, "STATUS") == 0 || strcmp(command, "?") == 0) {
        if (!console_has_extra(&cursor)) {
            debug_status();
        } else {
            console_write("ERR usage: STATUS\r\n");
        }
        return;
    }
    if (strcmp(command, "BKIN") == 0) {
        if (!console_has_extra(&cursor)) {
            debug_bkin();
        } else {
            console_write("ERR usage: BKIN\r\n");
        }
        return;
    }
    if (strcmp(command, "FAULT") == 0) {
        argument = console_next_token(&cursor);
        if (argument != NULL && strcmp(argument, "CLEAR") == 0 &&
            !console_has_extra(&cursor)) {
            console_clear_fault();
        } else {
            console_write("ERR usage: FAULT CLEAR\r\n");
        }
        return;
    }
    if (strcmp(command, "MODE") == 0) {
        argument = console_next_token(&cursor);
        if (argument == NULL || console_has_extra(&cursor)) {
            console_write("ERR usage: MODE CAN|UART\r\n");
        } else if (strcmp(argument, "UART") == 0) {
            if (uart_control_active == 0U) {
                Bldc_EmergencyStop();
                uart_motor_running = 0U;
                uart_control_active = 1U;
                console_write("OK control=UART; send MOTOR START to run\r\n");
            } else {
                console_write("OK already in UART mode\r\n");
            }
        } else if (strcmp(argument, "CAN") == 0) {
            if (uart_control_active != 0U) {
                uart_motor_running = 0U;
                uart_control_active = 0U;
                Bldc_EmergencyStop();
                console_write("OK UART target stopped; control=CAN, waiting for a CAN command\r\n");
            } else {
                console_write("OK already in CAN mode\r\n");
            }
        } else {
            console_write("ERR usage: MODE CAN|UART\r\n");
        }
        return;
    }
    if (strcmp(command, "MOTOR") == 0) {
        argument = console_next_token(&cursor);
        if (argument == NULL) {
            console_write("ERR usage: MOTOR START|STOP|SPEED n|DIR FWD|REV\r\n");
            return;
        }
        if (strcmp(argument, "START") == 0 && !console_has_extra(&cursor)) {
            select_uart_control();
#if MOTOR_OPEN_LOOP_TEST
            {
                BldcStatus status;
                Bldc_GetStatus(&status);
                if (status.fault != BLDC_FAULT_NONE) {
                    console_write("ERR motor fault latched; send MOTOR STOP then FAULT CLEAR\r\n");
                    return;
                }
            }
#endif
            if (uart_motor_speed_percent < 8U) {
                console_write("ERR motor speed is zero. ");
                console_motor_speed_range();
            } else {
                uart_motor_running = 1U;
                console_write("OK motor start requested\r\n");
            }
        } else if (strcmp(argument, "STOP") == 0 && !console_has_extra(&cursor)) {
            select_uart_control();
            uart_motor_running = 0U;
            Bldc_EmergencyStop();
            console_write("OK motor stopped\r\n");
        } else if (strcmp(argument, "SPEED") == 0) {
            argument = console_next_token(&cursor);
            if (argument == NULL || console_has_extra(&cursor) ||
                !console_parse_uint(argument, UART_MOTOR_MAX_PERCENT, &value) ||
                (value != 0U && value < 8U)) {
                console_write("ERR ");
                console_motor_speed_range();
                return;
            }
            select_uart_control();
            uart_motor_speed_percent = (uint8_t)value;
            if (value == 0U) {
                uart_motor_running = 0U;
                Bldc_EmergencyStop();
            }
            console_write("OK motor speed request updated\r\n");
        } else if (strcmp(argument, "DIR") == 0) {
            argument = console_next_token(&cursor);
            if (argument == NULL || console_has_extra(&cursor) ||
                (strcmp(argument, "FWD") != 0 && strcmp(argument, "REV") != 0)) {
                console_write("ERR direction must be FWD or REV\r\n");
                return;
            }
            select_uart_control();
            uart_motor_direction = (strcmp(argument, "REV") == 0) ? -1 : 1;
            console_write("OK motor direction request updated\r\n");
        } else {
            console_write("ERR usage: MOTOR START|STOP|SPEED n|DIR FWD|REV\r\n");
        }
        return;
    }
    if (strcmp(command, "DC") == 0) {
        argument = console_next_token(&cursor);
        if (argument == NULL) {
            console_write("ERR usage: DC SPEED 0..100|START|STOP|DIR FWD/REV\r\n");
            return;
        }
        if (strcmp(argument, "START") == 0 && !console_has_extra(&cursor)) {
            if (brushed_motor_speed_percent == 0U) {
                console_write("ERR set DC SPEED 1..100 first\r\n");
            } else {
                brushed_motor_running = 1U;
                brushed_motor_apply();
                console_write("OK DC start requested\r\n");
            }
        } else if (strcmp(argument, "STOP") == 0 && !console_has_extra(&cursor)) {
            brushed_motor_running = 0U;
            Actuators_Stop();
            console_write("OK DC stopped\r\n");
        } else if (strcmp(argument, "SPEED") == 0) {
            argument = console_next_token(&cursor);
            if (argument == NULL || console_has_extra(&cursor) ||
                !console_parse_uint(argument, 100U, &value)) {
                console_write("ERR DC speed must be 0..100 percent\r\n");
                return;
            }
            brushed_motor_speed_percent = (uint8_t)value;
            if (value == 0U) {
                brushed_motor_running = 0U;
            }
            brushed_motor_apply();
            console_write("OK DC speed target updated\r\n");
        } else if (strcmp(argument, "DIR") == 0) {
            argument = console_next_token(&cursor);
            if (argument == NULL || console_has_extra(&cursor) ||
                (strcmp(argument, "FWD") != 0 && strcmp(argument, "REV") != 0)) {
                console_write("ERR DC direction must be FWD or REV\r\n");
                return;
            }
            /* Never reverse a live H-bridge: remove PWM before changing direction. */
            brushed_motor_running = 0U;
            Actuators_Stop();
            brushed_motor_direction = (strcmp(argument, "REV") == 0) ? -1 : 1;
            console_write("OK DC direction set; motor stopped, send DC START\r\n");
        } else {
            console_write("ERR usage: DC SPEED 0..100|START|STOP|DIR FWD/REV\r\n");
        }
        return;
    }
    if (strcmp(command, "SERVO") == 0) {
        int16_t angle;
        argument = console_next_token(&cursor);
        if (argument != NULL && strcmp(argument, "ANGLE") == 0) {
            argument = console_next_token(&cursor);
        }
        if (argument == NULL || console_has_extra(&cursor) ||
            !console_parse_servo_angle(argument, &angle)) {
            console_write("ERR usage: SERVO ANGLE -60..60\r\n");
            return;
        }
        Servo_SetPulseUs((uint16_t)(1500 + (int32_t)angle * 10));
        console_write("OK servo angle request updated\r\n");
        return;
    }
    console_write("ERR unknown command; send HELP\r\n");
}

static void console_task(void)
{
    uint8_t character;
    if (HAL_UART_Receive(&huart3, &character, 1U, 0U) != HAL_OK) {
        return;
    }
    if (console_length == 0U && character == '?') {
        debug_status();
        return;
    }
    /* Preserve the legacy one-character fault-clear shortcut. */
    if (console_length == 0U && (character == 'c' || character == 'C')) {
        console_clear_fault();
        return;
    }
    if (character == '\r' || character == '\n') {
        if (console_overflow != 0U) {
            console_write("ERR command too long\r\n");
        } else if (console_length != 0U) {
            console_line[console_length] = '\0';
            console_process_line();
        }
        console_length = 0U;
        console_overflow = 0U;
        return;
    }
    if (character == 8U || character == 127U) {
        if (console_overflow != 0U) {
            console_overflow = 0U;
        } else if (console_length != 0U) {
            console_length--;
        }
        return;
    }
    if (character < 32U || character > 126U || console_overflow != 0U) {
        return;
    }
    if (console_length >= (CONSOLE_LINE_CAPACITY - 1U)) {
        console_overflow = 1U;
        return;
    }
    if (character >= 'a' && character <= 'z') {
        character = (uint8_t)(character - 'a' + 'A');
    }
    console_line[console_length++] = (char)character;
}

static void uart_motor_task(uint32_t now)
{
    BldcStatus status;
    uint32_t voltage_mv;
    int32_t signed_voltage_mv;

    if (uart_control_active == 0U || uart_motor_running == 0U ||
        uart_motor_speed_percent < 8U) {
        return;
    }
    Bldc_GetStatus(&status);
    if (status.voltage_ready == 0U || status.vbus_mv == 0U ||
        status.vbus_adc <= MOTOR_VBUS_UNDERVOLTAGE_ADC ||
        status.vbus_adc > MOTOR_VBUS_OVERVOLTAGE_ADC) {
        return;
    }
    voltage_mv = (status.vbus_mv * uart_motor_speed_percent + 99U) / 100U;
    if (voltage_mv > 32767U) {
        voltage_mv = 32767U;
    }
    signed_voltage_mv = (int32_t)voltage_mv * uart_motor_direction;
    Bldc_SetVoltageMv((int16_t)signed_voltage_mv, now);
}

static void fault_led_task(uint32_t now)
{
    BldcStatus status;
    uint32_t interval;
    Bldc_GetStatus(&status);
    if ((uint8_t)status.fault != led_fault) {
        led_fault = (uint8_t)status.fault;
        led_edge = 0U;
        led_changed_ms = now;
        HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
    }
    if (led_fault == 0U) {
        return;
    }
    interval = (led_edge == 0U) ? 2000U : 250U;
    if ((uint32_t)(now - led_changed_ms) < interval) {
        return;
    }
    led_changed_ms = now;
    if (led_edge >= (uint8_t)(2U * led_fault)) {
        led_edge = 0U;
        HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
    } else {
        led_edge++;
        HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin,
            (led_edge & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void hall_print_task(uint32_t now)
{
    BldcStatus status;
    char text[24];
    int length;

    Bldc_GetStatus(&status);
    if (status.open_loop_test == 0U || status.state != BLDC_RUNNING) {
        hall_print_active = 0U;
        return;
    }
    if (hall_print_active == 0U) {
        hall_print_active = 1U;
        last_hall_print_ms = now;
        return;
    }
    if ((uint32_t)(now - last_hall_print_ms) < 100U) {
        return;
    }
    last_hall_print_ms = now;
    length = snprintf(text, sizeof(text), "HALL=%u\r\n", (unsigned)status.hall);
    if (length > 0 && length < (int)sizeof(text)) {
        (void)console_transmit((const uint8_t *)text, (uint16_t)length);
    }
}

HAL_StatusTypeDef MotorApp_Init(void)
{
    static const uint8_t greeting[] =
        "F103 six-step ready; UART 115200 8N1. Send HELP for commands.\r\n";
    app_ready = 0U;
    console_length = 0U;
    console_overflow = 0U;
    uart_control_active = 0U;
    uart_motor_running = 0U;
    uart_motor_speed_percent = UART_MOTOR_DEFAULT_PERCENT;
    uart_motor_direction = 1;
    brushed_motor_running = 0U;
    brushed_motor_speed_percent = 0U;
    brushed_motor_direction = 1;
    hall_print_active = 0U;
    last_hall_print_ms = 0U;
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
    led_fault = 0U;
    led_edge = 0U;
    led_changed_ms = HAL_GetTick();
    if (RS485_Init() != HAL_OK || Bldc_Init() != HAL_OK || Actuators_Init() != HAL_OK ||
        CanBus_Init(MOTOR_CAN_NODE_ID) != HAL_OK) {
        MotorApp_EmergencyStop();
        return HAL_ERROR;
    }
    /* Do not report an encoder position until a valid Modbus sample arrives. */
    CanBus_SetFeedback(0, 0);
    Mt6v10_Init(HAL_GetTick());
    app_ready = 1U;
    (void)console_transmit(greeting, (uint16_t)(sizeof(greeting) - 1U));
    return HAL_OK;
}

void MotorApp_Process(void)
{
    int16_t voltage_mv;
    uint32_t received_ms;
    uint32_t now;
    Mt6v10Status encoder;
    if (app_ready == 0U) {
        return;
    }
    console_task();
    now = HAL_GetTick();
    Mt6v10_Task(now);
    if (Mt6v10_TakeUpdate(&encoder) != 0U) {
        CanBus_SetFeedback(encoder.angle_mrad, encoder.speed_rpm_x10);
    }
    if (CanBus_TakeCommand(&voltage_mv, &received_ms) != 0U) {
        if (uart_control_active == 0U) {
            /* Keep the ISR receive timestamp; a slow foreground cannot extend it. */
            Bldc_SetVoltageMv(voltage_mv, received_ms);
        }
    }
    now = HAL_GetTick();
    uart_motor_task(now);
    hall_print_task(now);
    CanBus_Task(now);
    fault_led_task(now);
}

void MotorApp_EmergencyStop(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    app_ready = 0U;
    brushed_motor_running = 0U;
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    TIM1->CCER = 0U;
    TIM1->DIER = 0U;
    TIM3->CCER = 0U;
    TIM3->CCR4 = 0U;
    GPIOA->BRR = DC_IN1_Pin | DC_IN2_Pin;
    GPIOB->BRR = RS485_DIR_Pin;
    TIM4->CCER = 0U;
    __set_PRIMASK(key);
}

/* Owned here, rather than by CubeMX: do not additionally generate this vector.
 * Direct UIF handling avoids HAL's shared TIM1 IRQ/callback dispatch overhead.
 * The Break vector uses HAL_TIM_IRQHandler, whose PeriodElapsed callback is
 * deliberately left unused, so Break can never recurse into FastTick. */
void TIM1_UP_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_UIF) != 0U && (TIM1->DIER & TIM_DIER_UIE) != 0U) {
        TIM1->SR = (uint16_t)~TIM_SR_UIF;
        Bldc_FastTick();
    }
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM1) {
        Bldc_BreakIrq();
    }
}
