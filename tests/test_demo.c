#include "unity.h"

#include "aegis/types.h"
#include "aegis_config.h"
#include "demo_api.h"
#include "sim/arm.h"

#include <math.h>
#include <string.h>

#define WARMUP_TICKS 300
#define AFTER_TICKS 400

void setUp(void)
{
    demo_arm_reset();
}

void tearDown(void)
{
}

static void run_ticks(int n)
{
    int i;

    for (i = 0; i < n; i++) {
        demo_arm_tick();
    }
}

static int abs_int(int x)
{
    if (x < 0) {
        return -x;
    }
    return x;
}

/* breaks the arm, then checks the gated side has zero duty on every later tick */
static void break_and_check_gated(int fault, int expected_check)
{
    int i;
    int j;

    run_ticks(WARMUP_TICKS);
    TEST_ASSERT_EQUAL_INT(0, demo_arm_tripped());
    demo_arm_break(fault);

    for (i = 0; i < AFTER_TICKS; i++) {
        demo_arm_tick();
        /* the joint that did not trip is told to stop one tick later */
        if (demo_arm_tripped() && demo_arm_tick_count() - 1 > demo_arm_trip_tick() + 1) {
            for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
                TEST_ASSERT_EQUAL_INT(0, demo_arm_duty(DEMO_SIDE_GATED, j));
                TEST_ASSERT_NOT_EQUAL(AEGIS_STATE_RUNNING, demo_arm_state(j));
            }
        }
    }

    TEST_ASSERT_EQUAL_INT(1, demo_arm_tripped());
    TEST_ASSERT_EQUAL_INT(expected_check, demo_arm_trip_fault());
    for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, demo_arm_speed(DEMO_SIDE_GATED, j));
    }
}

static void test_no_fault_both_arms_follow_the_sweep(void)
{
    int i;
    int j;

    for (i = 0; i < 2000; i++) {
        demo_arm_tick();
        for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
            float a = demo_arm_angle(DEMO_SIDE_NOGATE, j);
            float b = demo_arm_angle(DEMO_SIDE_GATED, j);
            TEST_ASSERT_FLOAT_WITHIN(0.02f, a, b);
        }
    }
    TEST_ASSERT_EQUAL_INT(0, demo_arm_tripped());
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, demo_arm_state(0));
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, demo_arm_state(1));
}

static void test_jam_gate_stops_nogate_keeps_pushing(void)
{
    break_and_check_gated(SIM_ARM_FAULT_JAM, AEGIS_FAULT_STALL);
    TEST_ASSERT_TRUE(abs_int(demo_arm_duty(DEMO_SIDE_NOGATE, AEGIS_ARM_FAULT_JOINT)) >= 600);
}

static void test_frozen_encoder_gate_stops_nogate_spins(void)
{
    break_and_check_gated(SIM_ARM_FAULT_SENSOR, AEGIS_FAULT_DISAGREE);
    TEST_ASSERT_TRUE(fabsf(demo_arm_speed(DEMO_SIDE_NOGATE, AEGIS_ARM_FAULT_JOINT)) > AEGIS_V_MAX_RAD_S);
}

static void test_silent_host_gate_stops_nogate_keeps_driving(void)
{
    break_and_check_gated(SIM_ARM_FAULT_SILENT, AEGIS_FAULT_HEARTBEAT);
    TEST_ASSERT_TRUE(demo_arm_duty(DEMO_SIDE_NOGATE, AEGIS_ARM_FAULT_JOINT) != 0);
    TEST_ASSERT_TRUE(fabsf(demo_arm_speed(DEMO_SIDE_NOGATE, AEGIS_ARM_FAULT_JOINT)) > 0.3f);
}

static void test_runaway_gate_stops_nogate_spins(void)
{
    break_and_check_gated(SIM_ARM_FAULT_RUNAWAY, AEGIS_FAULT_TRACKING);
    TEST_ASSERT_TRUE(fabsf(demo_arm_speed(DEMO_SIDE_NOGATE, AEGIS_ARM_FAULT_JOINT)) > AEGIS_V_MAX_RAD_S);
}

static void test_only_first_break_counts(void)
{
    run_ticks(WARMUP_TICKS);
    demo_arm_break(SIM_ARM_FAULT_JAM);
    demo_arm_break(SIM_ARM_FAULT_RUNAWAY);
    TEST_ASSERT_EQUAL_INT(SIM_ARM_FAULT_JAM, demo_arm_fault());
    demo_arm_reset();
    demo_arm_break(99);
    TEST_ASSERT_EQUAL_INT(SIM_ARM_FAULT_NONE, demo_arm_fault());
}

static void record_run(int fault, float *angles, int *duties)
{
    int i;
    int n = 0;

    demo_arm_reset();
    for (i = 0; i < WARMUP_TICKS + AFTER_TICKS; i++) {
        int side;
        int j;

        if (i == WARMUP_TICKS) {
            demo_arm_break(fault);
        }
        demo_arm_tick();
        for (side = 0; side < 2; side++) {
            for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
                angles[n] = demo_arm_angle(side, j);
                duties[n] = demo_arm_duty(side, j);
                n++;
            }
        }
    }
}

static void test_same_inputs_give_same_output(void)
{
    static float angles_a[(WARMUP_TICKS + AFTER_TICKS) * 2 * AEGIS_ARM_JOINTS];
    static float angles_b[(WARMUP_TICKS + AFTER_TICKS) * 2 * AEGIS_ARM_JOINTS];
    static int duties_a[(WARMUP_TICKS + AFTER_TICKS) * 2 * AEGIS_ARM_JOINTS];
    static int duties_b[(WARMUP_TICKS + AFTER_TICKS) * 2 * AEGIS_ARM_JOINTS];
    int fault;

    for (fault = SIM_ARM_FAULT_NONE; fault < SIM_ARM_FAULT_COUNT; fault++) {
        record_run(fault, angles_a, duties_a);
        record_run(fault, angles_b, duties_b);
        TEST_ASSERT_EQUAL_INT(0, memcmp(angles_a, angles_b, sizeof(angles_a)));
        TEST_ASSERT_EQUAL_INT(0, memcmp(duties_a, duties_b, sizeof(duties_a)));
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_fault_both_arms_follow_the_sweep);
    RUN_TEST(test_jam_gate_stops_nogate_keeps_pushing);
    RUN_TEST(test_frozen_encoder_gate_stops_nogate_spins);
    RUN_TEST(test_silent_host_gate_stops_nogate_keeps_driving);
    RUN_TEST(test_runaway_gate_stops_nogate_spins);
    RUN_TEST(test_only_first_break_counts);
    RUN_TEST(test_same_inputs_give_same_output);
    return UNITY_END();
}
