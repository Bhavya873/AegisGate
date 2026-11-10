#include "unity.h"

#include "aegis/types.h"
#include "aegis_config.h"
#include "demo_api.h"

void setUp(void)
{
    demo_reset();
}

void tearDown(void)
{
}

static void run_ticks(int n)
{
    int i;

    for (i = 0; i < n; i++) {
        demo_tick();
        while (demo_pop()) {
        }
    }
}

static void start_running(void)
{
    run_ticks(5);
    demo_send(DEMO_CMD_ARM);
    run_ticks(1);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, demo_state());
}

/* runs until the loop trips and returns how many ticks that took, or -1 */
static int ticks_until_trip(int max_ticks)
{
    int i;

    for (i = 1; i <= max_ticks; i++) {
        run_ticks(1);
        if (demo_state() == AEGIS_STATE_TRIPPED) {
            return i;
        }
    }
    return -1;
}

static void test_boots_armed_with_no_duty(void)
{
    run_ticks(20);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, demo_state());
    TEST_ASSERT_EQUAL_INT(0, demo_duty());
}

static void test_follows_target_without_tripping(void)
{
    start_running();
    demo_set_target(1.0f);
    run_ticks(600);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, demo_state());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, demo_angle());

    demo_set_target(-1.5f);
    run_ticks(1000);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, demo_state());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -1.5f, demo_angle());
}

static void test_heartbeat_cut_trips(void)
{
    start_running();
    demo_set_fault(DEMO_FAULT_HEARTBEAT, 1);
    TEST_ASSERT_TRUE(ticks_until_trip(50) > 0);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_HEARTBEAT, demo_fault());
}

static void test_jam_trips_stall_when_target_moves(void)
{
    start_running();
    demo_set_fault(DEMO_FAULT_JAM, 1);
    demo_set_target(1.0f);
    TEST_ASSERT_TRUE(ticks_until_trip(200) > 0);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_STALL, demo_fault());
}

static void test_gyro_bias_trips_disagree(void)
{
    start_running();
    demo_set_fault(DEMO_FAULT_GYRO_BIAS, 1);
    TEST_ASSERT_EQUAL_INT((int)AEGIS_DISAGREE_TICKS, ticks_until_trip(50));
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_DISAGREE, demo_fault());
}

static void test_spin_trips_velocity(void)
{
    start_running();
    demo_set_fault(DEMO_FAULT_SPIN, 1);
    TEST_ASSERT_EQUAL_INT(1, ticks_until_trip(50));
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_VELOCITY, demo_fault());
}

static void test_gyro_nan_trips_numeric(void)
{
    start_running();
    demo_set_fault(DEMO_FAULT_GYRO_NAN, 1);
    TEST_ASSERT_EQUAL_INT(1, ticks_until_trip(50));
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_NUMERIC, demo_fault());
}

static void test_ref_jump_trips_tracking(void)
{
    start_running();
    demo_jump_ref();
    TEST_ASSERT_TRUE(ticks_until_trip(50) > 0);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_TRACKING, demo_fault());
}

static void test_no_duty_after_trip_and_reset_goes_to_armed(void)
{
    int i;

    start_running();
    demo_set_target(1.0f);
    run_ticks(50);
    demo_set_fault(DEMO_FAULT_GYRO_BIAS, 1);
    TEST_ASSERT_TRUE(ticks_until_trip(50) > 0);
    demo_set_fault(DEMO_FAULT_GYRO_BIAS, 0);

    for (i = 0; i < 100; i++) {
        run_ticks(1);
        TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, demo_state());
        TEST_ASSERT_EQUAL_INT(0, demo_duty());
    }

    demo_send(DEMO_CMD_RESET);
    run_ticks(1);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, demo_state());
    run_ticks(20);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, demo_state());
    TEST_ASSERT_EQUAL_INT(0, demo_duty());

    demo_send(DEMO_CMD_ARM);
    run_ticks(1);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, demo_state());
}

static void test_cannot_arm_without_heartbeat(void)
{
    run_ticks(5);
    demo_set_fault(DEMO_FAULT_HEARTBEAT, 1);
    demo_send(DEMO_CMD_ARM);
    run_ticks(5);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, demo_state());
}

static void test_ring_drops_when_nobody_reads(void)
{
    int i;
    int got = 0;

    for (i = 0; i < 100; i++) {
        demo_tick();
    }
    while (demo_pop()) {
        got++;
    }
    TEST_ASSERT_EQUAL_INT(100, got + demo_dropped());
    TEST_ASSERT_TRUE(demo_dropped() > 0);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_boots_armed_with_no_duty);
    RUN_TEST(test_follows_target_without_tripping);
    RUN_TEST(test_heartbeat_cut_trips);
    RUN_TEST(test_jam_trips_stall_when_target_moves);
    RUN_TEST(test_gyro_bias_trips_disagree);
    RUN_TEST(test_spin_trips_velocity);
    RUN_TEST(test_gyro_nan_trips_numeric);
    RUN_TEST(test_ref_jump_trips_tracking);
    RUN_TEST(test_no_duty_after_trip_and_reset_goes_to_armed);
    RUN_TEST(test_cannot_arm_without_heartbeat);
    RUN_TEST(test_ring_drops_when_nobody_reads);
    return UNITY_END();
}
