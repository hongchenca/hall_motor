#include "motor_motion.h"
#include <string.h>

#define COUNTS_PER_TURN 65536LL
#define STOP_SPEED_X10 10L
#define STOP_SETTLE_MS 200U
#define STOP_TIMEOUT_MS 5000U
#define TRACK_GRACE_MS 1500U
#define TRACK_ERROR_MS 1000U

static MotorMotionStatus motion;
static uint8_t have_sample, supply_ready, driver_has_fault;
static uint8_t quiet_seen, bad_tracking;
static uint32_t clock_ms, sample_ms, raw_position;
static uint32_t wait_started_ms, quiet_ms, drive_started_ms, bad_tracking_ms;
static uint32_t move_started_ms, move_timeout_ms;
static int64_t position_counts;
static int32_t ramp, integral;
static int8_t applied_direction;

static int32_t absolute(int32_t value) { return value < 0 ? -value : value; }

static int32_t clamp(int32_t value, int32_t low, int32_t high)
{
    return value < low ? low : (value > high ? high : value);
}

static uint32_t square_root(uint32_t value)
{
    uint32_t result = 0U, bit = 1UL << 30;
    while (bit > value) { bit >>= 2; }
    while (bit != 0U) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else { result >>= 1; }
        bit >>= 2;
    }
    return result;
}

static void output_off(void)
{
    motion.drive_direction = 0;
    motion.interval_ticks = 0U;
    motion.ramp_rpm_x10 = 0;
    ramp = 0;
    integral = 0;
    bad_tracking = 0U;
}

static void fail(MotorMotionFault fault)
{
    output_off();
    motion.active = 0U;
    motion.fault = fault;
    motion.state = MOTION_FAULT;
}

static void wait_for_stop(void)
{
    output_off();
    motion.state = MOTION_WAIT_STOP;
    wait_started_ms = clock_ms;
    quiet_seen = 0U;
}

static void reset_move_timeout(void)
{
    uint32_t distance = (uint32_t)absolute(motion.target_deg_x10 -
                                          motion.position_deg_x10);
    move_timeout_ms = 30000U + (uint32_t)(((uint64_t)distance * 3000U) /
                                         (60U * motion.rpm_limit));
    move_started_ms = clock_ms;
}

void MotorMotion_Init(void)
{
    memset(&motion, 0, sizeof(motion));
    motion.direction = 1;
    motion.rpm_limit = MOTION_DEFAULT_RPM;
    have_sample = supply_ready = driver_has_fault = 0U;
    quiet_seen = bad_tracking = 0U;
    clock_ms = sample_ms = raw_position = 0U;
    position_counts = 0;
    applied_direction = 1;
    output_off();
}

void MotorMotion_Stop(void)
{
    output_off();
    motion.active = 0U;
    motion.state = motion.fault == MOTION_OK ? MOTION_IDLE : MOTION_FAULT;
}

uint8_t MotorMotion_SetRpm(uint16_t rpm)
{
    if (rpm != 0U && (rpm < MOTION_MIN_RPM || rpm > MOTION_MAX_RPM)) {
        return 0U;
    }
    /* Updating the position speed limit preserves a move already in progress. */
    motion.rpm_limit = rpm;
    if (rpm == 0U) { MotorMotion_Stop(); }
    else if (motion.active != 0U && motion.position_mode != 0U) {
        reset_move_timeout();
    }
    return 1U;
}

void MotorMotion_SetDirection(int8_t direction)
{
    if (motion.position_mode != 0U) { MotorMotion_Stop(); }
    motion.position_mode = 0U;
    motion.direction = direction < 0 ? -1 : 1;
    if (motion.active != 0U && motion.direction != applied_direction) {
        wait_for_stop();
    }
}

static uint8_t can_start(void)
{
    return (uint8_t)(motion.feedback_ready != 0U && supply_ready != 0U &&
        driver_has_fault == 0U && motion.fault == MOTION_OK &&
        motion.rpm_limit >= MOTION_MIN_RPM);
}

uint8_t MotorMotion_Start(void)
{
    if (can_start() == 0U) { return 0U; }
    if (motion.active != 0U && motion.position_mode == 0U) { return 1U; }
    motion.position_mode = 0U;
    motion.active = 1U;
    wait_for_stop();
    return 1U;
}

uint8_t MotorMotion_Zero(void)
{
    if (motion.active != 0U || motion.feedback_ready == 0U ||
        absolute(motion.speed_rpm_x10) > STOP_SPEED_X10) { return 0U; }
    position_counts = 0;
    motion.position_deg_x10 = 0;
    motion.target_deg_x10 = 0;
    motion.reference_valid = 1U;
    return 1U;
}

uint8_t MotorMotion_Move(int32_t degrees, uint8_t relative)
{
    int64_t target;
    if (can_start() == 0U || motion.reference_valid == 0U ||
        degrees < -MOTION_MAX_DEGREES || degrees > MOTION_MAX_DEGREES) {
        return 0U;
    }
    target = (int64_t)degrees * 10;
    if (relative != 0U) { target += motion.position_deg_x10; }
    if (target < -MOTION_MAX_DEGREES * 10LL ||
        target > MOTION_MAX_DEGREES * 10LL) { return 0U; }
    motion.target_deg_x10 = (int32_t)target;
    motion.position_mode = 1U;
    motion.active = 1U;
    reset_move_timeout();
    wait_for_stop();
    return 1U;
}

uint8_t MotorMotion_ClearFault(void)
{
    if (motion.active != 0U || motion.feedback_ready == 0U ||
        driver_has_fault != 0U || supply_ready == 0U) { return 0U; }
    motion.fault = MOTION_OK;
    motion.state = MOTION_IDLE;
    return 1U;
}

/* Returns 1 only for a fresh, plausible sample. Turn counter rollover is
 * unwrapped before the FWD polarity conversion or speed calculation. */
static uint8_t sample_encoder(const Mt6v10Status *encoder, uint32_t *elapsed)
{
    uint32_t raw, dt;
    int64_t delta, limit, position;
    int32_t speed;
    motion.feedback_ready = (uint8_t)(encoder->online != 0U &&
        encoder->has_sample != 0U &&
        (uint32_t)(clock_ms - encoder->sample_ms) < MOTION_ENCODER_TIMEOUT_MS);
    if (motion.feedback_ready == 0U) { return 0U; }
    raw = ((uint32_t)(uint16_t)encoder->turn_count << 16) | encoder->angle_raw;
    if (have_sample == 0U) {
        raw_position = raw;
        sample_ms = encoder->sample_ms;
        have_sample = 1U;
        motion.reference_valid = 1U;
        motion.speed_rpm_x10 = 0;
        *elapsed = 0U;
        return 1U;
    }
    dt = encoder->sample_ms - sample_ms;
    if (dt == 0U) { return 0U; }
    delta = (int64_t)raw - raw_position;
    if (delta > 2147483647LL) { delta -= 4294967296LL; }
    if (delta < -2147483648LL) { delta += 4294967296LL; }
    raw_position = raw;
    sample_ms = encoder->sample_ms;
    if (dt >= MOTION_ENCODER_TIMEOUT_MS && motion.active != 0U) {
        /* A fresh frame cannot hide a missed control deadline. */
        fail(MOTION_ENCODER_LOST);
    }
    /* Reject a reset/jump faster than 1200 output rpm (+1 degree quantization).
     * Rebase sampling, but require a new ZERO before using position again. */
    limit = (int64_t)dt * 1311 + 183;
    if (delta > limit || delta < -limit) {
        motion.reference_valid = 0U;
        motion.feedback_ready = 0U;
        if (motion.active != 0U) { fail(MOTION_ENCODER_JUMP); }
        return 0U;
    }
    delta *= MOTION_ENCODER_SIGN;
    position_counts += delta;
    position = position_counts * 3600 / COUNTS_PER_TURN;
    /* Leave room for continuous-speed operation (over 59,000 shaft turns).
     * At this software range limit invalidate position, but keep speed running. */
    if (position > 214000000LL || position < -214000000LL) {
        motion.reference_valid = 0U;
        if (motion.active != 0U && motion.position_mode != 0U) {
            fail(MOTION_ENCODER_JUMP);
        }
        position_counts = 0;
        position = 0;
    }
    motion.position_deg_x10 = (int32_t)position;
    speed = (int32_t)(delta * 600000LL / (COUNTS_PER_TURN * dt));
    /* A new sample after a gap must not inherit stale filtered velocity. */
    if (dt >= MOTION_ENCODER_TIMEOUT_MS) { motion.speed_rpm_x10 = speed; }
    else { motion.speed_rpm_x10 += (speed - motion.speed_rpm_x10) / 4; }
    *elapsed = dt;
    return 1U;
}

void MotorMotion_Update(uint32_t now, const Mt6v10Status *encoder,
                        uint8_t driver_ready, uint8_t driver_fault)
{
    uint32_t dt = 0U;
    uint8_t fresh;
    int32_t error, distance, wanted, cap, measured, correction, frequency_rpm;
    int8_t direction;
    clock_ms = now;
    supply_ready = driver_ready;
    driver_has_fault = driver_fault;
    fresh = sample_encoder(encoder, &dt);
    if (motion.active == 0U) { return; }
    if (driver_has_fault != 0U || supply_ready == 0U) {
        fail(MOTION_DRIVER_FAULT); return;
    }
    if (motion.feedback_ready == 0U) { fail(MOTION_ENCODER_LOST); return; }
    if (motion.position_mode != 0U &&
        (uint32_t)(now - move_started_ms) >= move_timeout_ms) {
        fail(MOTION_MOVE_TIMEOUT); return;
    }
    if (fresh == 0U || dt == 0U) { return; }
    error = motion.target_deg_x10 - motion.position_deg_x10;
    distance = absolute(error);
    direction = motion.position_mode != 0U ? (error < 0 ? -1 : 1) : motion.direction;

    if (motion.position_mode != 0U &&
        distance <= MOTION_POSITION_TOLERANCE_X10 &&
        motion.state != MOTION_WAIT_STOP && motion.state != MOTION_SETTLING) {
        output_off();
        motion.state = MOTION_SETTLING;
        wait_started_ms = now;
        quiet_seen = 0U;
    }
    if (motion.state == MOTION_WAIT_STOP || motion.state == MOTION_SETTLING) {
        if ((uint32_t)(now - wait_started_ms) >= STOP_TIMEOUT_MS) {
            fail(MOTION_STOP_TIMEOUT); return;
        }
        if (absolute(motion.speed_rpm_x10) > STOP_SPEED_X10) {
            quiet_seen = 0U;
            return;
        }
        if (quiet_seen == 0U) { quiet_seen = 1U; quiet_ms = now; }
        if ((uint32_t)(now - quiet_ms) < STOP_SETTLE_MS) { return; }
        if (motion.position_mode != 0U && distance <= MOTION_POSITION_TOLERANCE_X10) {
            output_off(); motion.active = 0U; motion.state = MOTION_DONE; return;
        }
        applied_direction = direction;
        ramp = MOTION_MIN_RPM * 10;
        integral = 0;
        drive_started_ms = now;
        motion.state = motion.position_mode != 0U ? MOTION_POSITION : MOTION_SPEED;
    }
    if (direction != applied_direction) { wait_for_stop(); return; }

    wanted = (int32_t)motion.rpm_limit * 10;
    if (motion.position_mode != 0U) {
        /* Conservative stopping-distance bound plus proportional approach.
         * Real gearbox coast/backlash still determines achievable accuracy. */
        cap = (int32_t)square_root((uint32_t)((int64_t)distance *
                                             MOTION_ACCEL_RPM_X10_PER_S / 6));
        if (wanted > cap) { wanted = cap; }
        cap = distance / 3;
        if (wanted > cap) { wanted = cap; }
        if (wanted < MOTION_MIN_RPM * 10) { wanted = MOTION_MIN_RPM * 10; }
    }
    cap = (int32_t)(MOTION_ACCEL_RPM_X10_PER_S * dt / 1000U);
    if (cap < 1) { cap = 1; }
    ramp += clamp(wanted - ramp, -cap, cap);
    measured = motion.speed_rpm_x10 * applied_direction;
    error = ramp - measured;
    /* PI trims only +/-20% around the measured baseline frequency scale.
     * Sustained slip stalls the command instead of accelerating without bound. */
    cap = ramp / 5;
    integral = clamp(integral + error * (int32_t)dt / 2, -cap * 1000, cap * 1000);
    correction = clamp(error / 4 + integral / 1000, -cap, cap);
    frequency_rpm = ramp + correction;
    motion.interval_ticks = (uint16_t)clamp(
        (MOTION_BASE_INTERVAL_TICKS * MOTION_BASE_RPM_X10 + frequency_rpm / 2) /
        frequency_rpm, 8, 2000);
    motion.drive_direction = applied_direction;
    motion.ramp_rpm_x10 = ramp * applied_direction;

    if ((uint32_t)(now - drive_started_ms) >= TRACK_GRACE_MS &&
        (measured < -STOP_SPEED_X10 || absolute(error) > (ramp / 2 + 10))) {
        if (bad_tracking == 0U) { bad_tracking = 1U; bad_tracking_ms = now; }
        if ((uint32_t)(now - bad_tracking_ms) >= TRACK_ERROR_MS) {
            fail(MOTION_TRACKING_ERROR);
        }
    } else { bad_tracking = 0U; }
}

void MotorMotion_GetStatus(MotorMotionStatus *status)
{
    if (status != 0) { *status = motion; }
}
