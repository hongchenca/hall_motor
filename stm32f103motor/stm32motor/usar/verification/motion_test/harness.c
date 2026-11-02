/* Only the input/output boundary. The production controller is linked intact. */
#include "motor_motion.h"
#include <string.h>

static Mt6v10Status encoder;
int32_t test_snapshot[14];

void Test_Reset(void)
{
    memset(&encoder, 0, sizeof(encoder));
    MotorMotion_Init();
}

/* raw combines the signed multi-turn register and the 16-bit angle register. */
void Test_Encoder(uint32_t raw, uint32_t sample_ms, uint32_t flags)
{
    encoder.angle_raw = (uint16_t)raw;
    encoder.turn_count = (int16_t)(raw >> 16);
    encoder.sample_ms = sample_ms;
    encoder.has_sample = (uint8_t)(flags & 1U);
    encoder.online = (uint8_t)((flags >> 1) & 1U);
    /* Deliberately contradictory: motion must derive speed from raw position. */
    encoder.speed_rpm_x10 = 1234;
}

void Test_Update(uint32_t now, uint32_t ready, uint32_t fault)
{
    MotorMotion_Update(now, &encoder, (uint8_t)ready, (uint8_t)fault);
}

void Test_Snapshot(void)
{
    MotorMotionStatus s;
    MotorMotion_GetStatus(&s);
    test_snapshot[0] = s.state;
    test_snapshot[1] = s.fault;
    test_snapshot[2] = s.active;
    test_snapshot[3] = s.position_mode;
    test_snapshot[4] = s.feedback_ready;
    test_snapshot[5] = s.reference_valid;
    test_snapshot[6] = s.direction;
    test_snapshot[7] = s.drive_direction;
    test_snapshot[8] = s.rpm_limit;
    test_snapshot[9] = s.interval_ticks;
    test_snapshot[10] = s.speed_rpm_x10;
    test_snapshot[11] = s.ramp_rpm_x10;
    test_snapshot[12] = s.position_deg_x10;
    test_snapshot[13] = s.target_deg_x10;
}

void test_entry(void) { Test_Reset(); }
