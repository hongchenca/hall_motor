/* Adapted from Source/Bldc.c, Source/Protect.c and Include/Define.h.
 * Original algorithm/copyright: ZhangFeiShiZhanDianZi.
 * F103 HAL startup, ADC scheduling, bidirectional CAN command and fault latch port. */
#include "bldc_motor.h"
#include "motor_config.h"
#include "adc.h"
#include "tim.h"
#include <string.h>

typedef struct {
    uint16_t ccer;
    uint16_t ccmr1;
    uint16_t ccmr2;
} Phase;

/* Hall code bit order is PB0:PA7:PA6. One PWM high side, one different
 * constant low side. Channel 4 stays disabled and retains its CubeMX mode. */
static const Phase phases[7] = {
    {0x0000U, 0x0000U, 0x0000U},
    {0x0140U, 0x5000U, 0x0068U}, /* W+ V- */
    {0x0014U, 0x6850U, 0x0000U}, /* V+ U- */
    {0x0104U, 0x0050U, 0x0068U}, /* W+ U- */
    {0x0401U, 0x0068U, 0x0050U}, /* U+ W- */
    {0x0041U, 0x5068U, 0x0000U}, /* U+ V- */
    {0x0410U, 0x6800U, 0x0050U}  /* V+ W- */
};
#if MOTOR_OPEN_LOOP_TEST
static const uint8_t open_loop_phases[6] = {5U, 4U, 6U, 2U, 3U, 1U};
#else
static const uint8_t reverse_phase[7] = {0U, 6U, 5U, 4U, 3U, 2U, 1U};
#endif

static volatile BldcStatus motor;
static volatile BldcBreakDiagnostic break_diagnostic;
static volatile int16_t requested_mv;
static volatile uint32_t command_ms;
static volatile uint8_t command_seen;
/* DMA rank order: PA1 bus voltage, then PA2 auxiliary analog input.
 * Continuous conversion must stay disabled: FastTick starts the next pair
 * only after consuming this one, so DMA cannot overwrite a pair being read. */
__ALIGN_BEGIN static volatile uint16_t adc_data[2] __ALIGN_END;
static volatile uint8_t adc_pair_ready;
static volatile uint8_t adc_dma_error;
static uint32_t adc_sum;
static uint16_t adc_samples;
static uint16_t adc_missing_ticks;
static uint16_t state_ticks;
#if MOTOR_OPEN_LOOP_TEST
static uint16_t commutation_ticks;
static volatile uint16_t requested_commutation_ticks;
#else
static uint32_t stall_ticks;
static uint8_t previous_hall;
#endif
static uint8_t millisecond_divider;
static uint8_t voltage_bad_ms;
static int8_t direction;

static void delay_us(uint16_t us)
{
    uint16_t start = (uint16_t)TIM2->CNT;
    while ((uint16_t)((uint16_t)TIM2->CNT - start) < us) {
        /* TIM2 is a free-running 1 MHz counter, shared by no other module. */
    }
}

static uint8_t read_hall(void)
{
    uint32_t a = GPIOA->IDR;
    uint32_t b = GPIOB->IDR;
    uint8_t pa6 = (uint8_t)((a >> 6U) & 1U);
    uint8_t pa7 = (uint8_t)((a >> 7U) & 1U);
    uint8_t pb0 = (uint8_t)(b & 1U);
    return (uint8_t)((pb0 << 2U) | (pa7 << 1U) | pa6);
}

#if !MOTOR_OPEN_LOOP_TEST
static uint8_t read_stable_hall(uint8_t *hall)
{
    uint8_t a = read_hall();
    uint8_t b;
    delay_us(1U);
    b = read_hall();
    delay_us(1U);
    *hall = read_hall();
    return (uint8_t)((a == b) && (b == *hall));
}
#endif

static void outputs_off(void)
{
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    TIM1->CCER = 0U;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    motor.duty_ticks = 0U;
#if MOTOR_OPEN_LOOP_TEST
    motor.commutation_step = 0U;
    motor.commutation_interval_ticks = 0U;
    commutation_ticks = 0U;
#endif
}

static void latch_fault(BldcFault fault)
{
    outputs_off();
    if (motor.fault == BLDC_FAULT_NONE) {
        motor.fault = fault;
    }
    motor.state = BLDC_STOPPED;
}

/* Call with interrupts masked. Preserve the first evidence until a successful
 * explicit clear, including a short pulse that is high again by the callback. */
static void save_break_snapshot(BldcBreakSource source)
{
    if (motor.fault == BLDC_FAULT_NONE &&
        break_diagnostic.source == BLDC_BREAK_NONE) {
        break_diagnostic.sr = TIM1->SR;
        break_diagnostic.bdtr = TIM1->BDTR;
        break_diagnostic.dier = TIM1->DIER;
        break_diagnostic.pb12 = (uint8_t)((GPIOB->IDR & GPIO_PIN_12) != 0U);
        break_diagnostic.hall = read_hall();
        break_diagnostic.state = (uint8_t)motor.state;
        break_diagnostic.source = source;
    }
}

static void latch_break(BldcBreakSource source)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    save_break_snapshot(source);
    latch_fault(BLDC_FAULT_BREAK);
    __set_PRIMASK(key);
}

static void stop_motor(void)
{
    outputs_off();
    motor.state = BLDC_STOPPED;
#if !MOTOR_OPEN_LOOP_TEST
    previous_hall = 0U;
    stall_ticks = 0U;
#endif
    state_ticks = 0U;
}

static uint8_t enable_outputs(void)
{
    uint32_t key = __get_PRIMASK();
    uint8_t enabled = 0U;
    BldcBreakSource source = BLDC_BREAK_ENABLE_PRECHECK;
    __disable_irq();
    if ((motor.fault == BLDC_FAULT_NONE) &&
        ((GPIOB->IDR & GPIO_PIN_12) != 0U) &&
        ((TIM1->SR & TIM_SR_BIF) == 0U)) {
        SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
        /* RM0008: MOE is written asynchronously but read synchronously.
         * Allow readback to settle; BKIN remains enabled during this wait. */
        delay_us(1U);
        source = BLDC_BREAK_ENABLE_POSTCHECK;
        /* Hardware break always overrides MOE, including a transient here. */
        if (((TIM1->BDTR & TIM_BDTR_MOE) != 0U) &&
            ((TIM1->SR & TIM_SR_BIF) == 0U) &&
            ((GPIOB->IDR & GPIO_PIN_12) != 0U)) {
            enabled = 1U;
        }
    }
    if (enabled == 0U) {
        latch_break(source);
    }
    __set_PRIMASK(key);
    return enabled;
}

static void apply_phase(uint8_t index)
{
#if MOTOR_OPEN_LOOP_TEST
    const Phase *phase = &phases[index];
#else
    const Phase *phase = &phases[(direction > 0) ? index : reverse_phase[index]];
#endif
    /* Disconnect every channel before changing forced/PWM output modes.
     * Gate-driver inputs must have pull-downs for the disconnected interval. */
    TIM1->CCER = 0U;
    delay_us(1U);
    TIM1->CCMR1 = phase->ccmr1;
    TIM1->CCMR2 = (TIM1->CCMR2 & 0xFF00U) | phase->ccmr2;
    TIM1->CCER = phase->ccer;
    /* Never re-enable MOE here: a higher-priority Break may have cleared it. */
}

#if MOTOR_OPEN_LOOP_TEST
/* Hall is diagnostic only. Latch a requested period at each phase boundary. */
static void open_loop_tick(int8_t requested_direction, uint16_t duty)
{
    uint8_t next_step;

    if (motor.state == BLDC_RUNNING && requested_direction != direction) {
        outputs_off();
        motor.state = BLDC_COASTING;
        state_ticks = MOTOR_REVERSE_COAST_TICKS;
        return;
    }
    if (motor.state == BLDC_COASTING) {
        if (state_ticks != 0U) {
            state_ticks--;
        }
        if (state_ticks != 0U) {
            return;
        }
        motor.state = BLDC_STOPPED;
    }
    if (motor.state == BLDC_STOPPED) {
        if ((motor.vbus_adc <= MOTOR_VBUS_UNDERVOLTAGE_ADC) ||
            (motor.vbus_adc > MOTOR_VBUS_OVERVOLTAGE_ADC)) {
            return;
        }
        direction = requested_direction;
        TIM1->CCER = 0U;
        TIM1->CCR1 = duty;
        TIM1->CCR2 = duty;
        TIM1->CCR3 = duty;
        TIM1->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        motor.duty_ticks = duty;
        motor.commutation_step = 1U;
        motor.commutation_interval_ticks = requested_commutation_ticks;
        commutation_ticks = motor.commutation_interval_ticks;
        motor.state = BLDC_RUNNING;
        apply_phase(open_loop_phases[0]);
        (void)enable_outputs();
        return;
    }
    motor.duty_ticks = duty;
    TIM1->CCR1 = duty;
    TIM1->CCR2 = duty;
    TIM1->CCR3 = duty;
    if (motor.state == BLDC_RUNNING) {
        if (commutation_ticks != 0U) {
            commutation_ticks--;
        }
        if (commutation_ticks != 0U) {
            return;
        }
        next_step = motor.commutation_step;
        if (direction > 0) {
            next_step = (next_step >= 6U) ? 1U : (uint8_t)(next_step + 1U);
        } else {
            next_step = (next_step <= 1U) ? 6U : (uint8_t)(next_step - 1U);
        }
        motor.commutation_step = next_step;
        motor.commutation_interval_ticks = requested_commutation_ticks;
        commutation_ticks = motor.commutation_interval_ticks;
        apply_phase(open_loop_phases[next_step - 1U]);
    }
}
#endif

static void sample_voltage(void)
{
    if (adc_dma_error != 0U) {
        motor.voltage_ready = 0U;
        latch_fault(BLDC_FAULT_ADC);
        return;
    }
    if (adc_pair_ready != 0U) {
        adc_pair_ready = 0U;
        adc_sum += adc_data[0];
        motor.pa2_adc = adc_data[1];
        adc_missing_ticks = 0U;
        adc_samples++;
        if (adc_samples == MOTOR_ADC_FILTER_SAMPLES) {
            uint16_t average = (uint16_t)(adc_sum / MOTOR_ADC_FILTER_SAMPLES);
            motor.vbus_adc = average;
            motor.vbus_mv = ((uint32_t)average * MOTOR_VBUS_REFERENCE_MV) /
                            MOTOR_VBUS_REFERENCE_ADC;
            motor.voltage_ready = 1U;
            adc_sum = 0U;
            adc_samples = 0U;
        }
        /* One two-channel scan per trigger; circular DMA remains armed.
         * Never read DR here: DMA owns both conversion results. */
        SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
    } else {
        if (adc_missing_ticks < MOTOR_ADC_TIMEOUT_TICKS) {
            adc_missing_ticks++;
        } else {
            motor.voltage_ready = 0U;
            adc_sum = 0U;
            adc_samples = 0U;
            latch_fault(BLDC_FAULT_ADC);
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc->Instance == ADC1 && adc_dma_error == 0U) {
        /* DMA transfer complete means both halfwords have reached memory. */
        adc_pair_ready = 1U;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    if (adc->Instance == ADC1) {
        /* A DMA transfer error requires reinitialization, not fault clear. */
        adc_dma_error = 1U;
        adc_pair_ready = 0U;
        motor.voltage_ready = 0U;
        latch_fault(BLDC_FAULT_ADC);
    }
}

static void voltage_protection_1ms(void)
{
    BldcFault fault;
    if (motor.voltage_ready == 0U) {
        return;
    }
    if (motor.vbus_adc > MOTOR_VBUS_OVERVOLTAGE_ADC) {
        fault = BLDC_FAULT_OVERVOLTAGE;
    } else if (motor.vbus_adc <= MOTOR_VBUS_UNDERVOLTAGE_ADC) {
        fault = BLDC_FAULT_UNDERVOLTAGE;
    } else {
        voltage_bad_ms = 0U;
        return;
    }
    if (voltage_bad_ms < MOTOR_VBUS_CONFIRM_MS) {
        voltage_bad_ms++;
    }
    if (voltage_bad_ms >= MOTOR_VBUS_CONFIRM_MS) {
        latch_fault(fault);
    }
}

HAL_StatusTypeDef Bldc_Init(void)
{
    memset((void *)&motor, 0, sizeof(motor));
    motor.open_loop_test = MOTOR_OPEN_LOOP_TEST;
#if MOTOR_OPEN_LOOP_TEST
    motor.hall = read_hall();
    requested_commutation_ticks = MOTOR_OPEN_LOOP_COMMUTATION_TICKS;
#endif
    memset((void *)&break_diagnostic, 0, sizeof(break_diagnostic));
    requested_mv = 0;
    command_seen = 0U;
    command_ms = 0U;
    adc_data[0] = 0U;
    adc_data[1] = 0U;
    adc_pair_ready = 0U;
    adc_dma_error = 0U;
    adc_sum = 0U;
    adc_samples = 0U;
    adc_missing_ticks = 0U;
    state_ticks = 0U;
#if !MOTOR_OPEN_LOOP_TEST
    stall_ticks = 0U;
    previous_hall = 0U;
#endif
    millisecond_divider = 0U;
    voltage_bad_ms = 0U;
    direction = 1;
    outputs_off();
    /* Inactive/off-state outputs low; do not change CubeMX Break polarity. */
    CLEAR_BIT(TIM1->CR2, TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 |
              TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N | TIM_CR2_CCPC);
    SET_BIT(TIM1->BDTR, TIM_BDTR_OSSI | TIM_BDTR_OSSR);
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE);
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
        return HAL_ERROR;
    }
    if ((HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) ||
        (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data, 2U) != HAL_OK)) {
        latch_fault(BLDC_FAULT_ADC);
        return HAL_ERROR;
    }
    /* Half-transfer contains PA1 only; process full PA1/PA2 pairs instead. */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    TIM1->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_BREAK);
    if ((GPIOB->IDR & GPIO_PIN_12) == 0U) {
        latch_break(BLDC_BREAK_INIT_PIN_LOW);
        __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_BREAK);
    } else {
        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    }
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
        outputs_off();
        return HAL_ERROR;
    }
    return HAL_OK;
}

void Bldc_SetVoltageMv(int16_t voltage_mv, uint32_t received_ms)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    requested_mv = voltage_mv;
    command_ms = received_ms;
    command_seen = 1U;
    __set_PRIMASK(key);
}

void Bldc_SetCommutationTicks(uint16_t ticks)
{
#if MOTOR_OPEN_LOOP_TEST
    uint32_t key;
    if (ticks < MOTOR_OPEN_LOOP_MIN_COMMUTATION_TICKS) {
        ticks = MOTOR_OPEN_LOOP_MIN_COMMUTATION_TICKS;
    } else if (ticks > MOTOR_OPEN_LOOP_MAX_COMMUTATION_TICKS) {
        ticks = MOTOR_OPEN_LOOP_MAX_COMMUTATION_TICKS;
    }
    key = __get_PRIMASK();
    __disable_irq();
    requested_commutation_ticks = ticks;
    __set_PRIMASK(key);
#else
    (void)ticks;
#endif
}

void Bldc_FastTick(void)
{
    uint32_t now = HAL_GetTick();
    int32_t voltage = requested_mv;
    int8_t requested_direction = (voltage < 0) ? -1 : 1;
    uint32_t duty;
#if !MOTOR_OPEN_LOOP_TEST
    uint8_t hall;
#else
    motor.hall = read_hall();
#endif

    sample_voltage();
    if (++millisecond_divider >= 20U) {
        millisecond_divider = 0U;
        voltage_protection_1ms();
    }
    motor.command_timed_out = (uint8_t)((command_seen == 0U) ||
        ((uint32_t)(now - command_ms) >= MOTOR_COMMAND_TIMEOUT_MS));
    if ((GPIOB->IDR & GPIO_PIN_12) == 0U) {
        latch_break(BLDC_BREAK_TICK_PIN_LOW);
    }
    if ((motor.fault != BLDC_FAULT_NONE) || (motor.voltage_ready == 0U) ||
        (motor.vbus_mv == 0U) ||
        (motor.command_timed_out != 0U) || (voltage == 0)) {
        stop_motor();
        return;
    }
    if (voltage < 0) {
        voltage = -voltage; /* int32 avoids overflow for CAN command -32768. */
    }
    duty = ((uint32_t)voltage * MOTOR_PWM_TICKS) / motor.vbus_mv;
    if (duty > MOTOR_MAX_DUTY_TICKS) {
        duty = MOTOR_MAX_DUTY_TICKS;
    }
#if MOTOR_OPEN_LOOP_TEST
    /* Bound duty for every caller, including CAN commands. */
    if (duty > MOTOR_OPEN_LOOP_MAX_DUTY_TICKS) {
        duty = MOTOR_OPEN_LOOP_MAX_DUTY_TICKS;
    }
#endif
    if (duty < MOTOR_MIN_DUTY_TICKS) {
        stop_motor();
        return;
    }
#if MOTOR_OPEN_LOOP_TEST
    open_loop_tick(requested_direction, (uint16_t)duty);
#else
    if ((motor.state == BLDC_RUNNING || motor.state == BLDC_PRECHARGE) &&
        (requested_direction != direction)) {
        outputs_off();
        motor.state = BLDC_COASTING;
        state_ticks = MOTOR_REVERSE_COAST_TICKS;
        return;
    }
    if (motor.state == BLDC_COASTING) {
        if (state_ticks != 0U) {
            state_ticks--;
            return;
        }
        motor.state = BLDC_STOPPED;
    }
    if (motor.state == BLDC_STOPPED) {
        /* Never start while the filtered supply is outside its valid range. */
        if ((motor.vbus_adc <= MOTOR_VBUS_UNDERVOLTAGE_ADC) ||
            (motor.vbus_adc > MOTOR_VBUS_OVERVOLTAGE_ADC)) {
            return;
        }
        direction = requested_direction;
        previous_hall = 0U;
        stall_ticks = 0U;
        TIM1->CCER = 0U;
        TIM1->CCMR1 = 0x5050U;
        TIM1->CCMR2 = (TIM1->CCMR2 & 0xFF00U) | 0x0050U;
        TIM1->CCER = 0x0444U; /* original three low-side startup */
        state_ticks = MOTOR_PRECHARGE_TICKS;
        motor.state = BLDC_PRECHARGE;
        if (enable_outputs() == 0U) {
            return;
        }
        return;
    }
    if (motor.state == BLDC_PRECHARGE && state_ticks != 0U) {
        state_ticks--;
        return;
    }
    if (read_stable_hall(&hall) == 0U) {
        if (++stall_ticks >= MOTOR_STALL_TIMEOUT_TICKS) {
            latch_fault(BLDC_FAULT_HALL);
        }
        return;
    }
    if (hall == 0U || hall == 7U) {
        latch_fault(BLDC_FAULT_HALL);
        return;
    }
    motor.hall = hall;
    motor.duty_ticks = (uint16_t)duty;
    TIM1->CCR1 = duty;
    TIM1->CCR2 = duty;
    TIM1->CCR3 = duty;
    if (motor.state == BLDC_PRECHARGE) {
        /* End precharge without applying a stale minimum-duty preload. */
        TIM1->CCER = 0U;
        TIM1->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        motor.state = BLDC_RUNNING;
    }
    if (hall != previous_hall) {
        apply_phase(hall);
        previous_hall = hall;
        stall_ticks = 0U;
    } else if (++stall_ticks >= MOTOR_STALL_TIMEOUT_TICKS) {
        latch_fault(BLDC_FAULT_HALL);
    }
#endif
    /* An IRQ may have preempted above. Keep the software state off as well. */
    if (motor.fault != BLDC_FAULT_NONE) {
        stop_motor();
    }
}

void Bldc_BreakIrq(void)
{
    /* Hardware has already cleared MOE. Latch even a short BKIN pulse. */
    latch_break(BLDC_BREAK_IRQ);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_BREAK);
}

void Bldc_CaptureBreakIrq(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if ((TIM1->SR & TIM_SR_BIF) != 0U &&
        (TIM1->DIER & TIM_IT_BREAK) != 0U) {
        save_break_snapshot(BLDC_BREAK_IRQ);
    }
    __set_PRIMASK(key);
}

void Bldc_GetBreakDiagnostic(BldcBreakDiagnostic *diagnostic)
{
    uint32_t key;
    if (diagnostic == NULL) {
        return;
    }
    key = __get_PRIMASK();
    __disable_irq();
    *diagnostic = break_diagnostic;
    __set_PRIMASK(key);
}

void Bldc_GetStatus(BldcStatus *status)
{
    uint32_t key;
    if (status == NULL) {
        return;
    }
    key = __get_PRIMASK();
    __disable_irq();
    *status = motor;
    __set_PRIMASK(key);
}

HAL_StatusTypeDef Bldc_ClearFault(void)
{
    uint32_t key = __get_PRIMASK();
#if !MOTOR_OPEN_LOOP_TEST
    uint8_t hall;
#endif
    __disable_irq();
#if !MOTOR_OPEN_LOOP_TEST
    hall = read_hall();
#endif
    if (requested_mv != 0 || motor.voltage_ready == 0U || adc_dma_error != 0U ||
        motor.vbus_adc <= MOTOR_VBUS_UNDERVOLTAGE_ADC ||
        motor.vbus_adc > MOTOR_VBUS_OVERVOLTAGE_ADC ||
        (GPIOB->IDR & GPIO_PIN_12) == 0U
#if !MOTOR_OPEN_LOOP_TEST
        || hall == 0U || hall == 7U
#endif
        ) {
        __set_PRIMASK(key);
        return HAL_ERROR;
    }
    stop_motor();
    motor.fault = BLDC_FAULT_NONE;
#if MOTOR_OPEN_LOOP_TEST
    motor.hall = read_hall();
#endif
    voltage_bad_ms = 0U;
    memset((void *)&break_diagnostic, 0, sizeof(break_diagnostic));
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    __set_PRIMASK(key);
    return HAL_OK;
}

void Bldc_EmergencyStop(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    requested_mv = 0;
#if MOTOR_OPEN_LOOP_TEST
    requested_commutation_ticks = MOTOR_OPEN_LOOP_COMMUTATION_TICKS;
#endif
    stop_motor();
    __set_PRIMASK(key);
}
