#include "unity.h"

#include "aegis/arbiter.h"
#include "aegis/safety.h"
#include "aegis/spsc.h"
#include "aegis/state_machine.h"
#include "aegis_config.h"
#include "sim/lockstep.h"

#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <string.h>

void setUp(void)
{
}

void tearDown(void)
{
}

static void clear_in(aegis_inputs_t *in, uint64_t now_us, int32_t enc, float gyro)
{
    memset(in, 0, sizeof(*in));
    in->now_us = now_us;
    in->enc_count = enc;
    in->gyro_rad_s = gyro;
    in->cmd = AEGIS_CMD_NONE;
}

static void boot_armed(aegis_ctx_t *ctx, uint64_t *now)
{
    aegis_inputs_t in;
    aegis_outputs_t out;

    aegis_init(ctx);
    clear_in(&in, *now, 0, 0.f);
    in.hb_valid = true;
    aegis_step(ctx, &in, &out);
    *now += AEGIS_PERIOD_US;
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, ctx->sm.state);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void arm_running(aegis_ctx_t *ctx, uint64_t *now)
{
    aegis_inputs_t in;
    aegis_outputs_t out;

    boot_armed(ctx, now);
    clear_in(&in, *now, 0, 0.f);
    in.hb_valid = true;
    in.cmd = AEGIS_CMD_ARM;
    aegis_step(ctx, &in, &out);
    *now += AEGIS_PERIOD_US;
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx->sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_NONE, ctx->first_fault);
}

static aegis_outputs_t step_traj(aegis_ctx_t *ctx, uint64_t *now, int32_t enc, float gyro, float ref,
                                 bool hb)
{
    aegis_inputs_t in;
    aegis_outputs_t out;

    clear_in(&in, *now, enc, gyro);
    in.hb_valid = hb;
    in.cmd = AEGIS_CMD_TRAJ;
    in.ref_pos_rad = ref;
    in.ref_vel_rad_s = 0.f;
    aegis_step(ctx, &in, &out);
    *now += AEGIS_PERIOD_US;
    return out;
}

static void test_zero_actuation_is_safe(void)
{
    aegis_actuation_t act;

    memset(&act, 0, sizeof(act));
    TEST_ASSERT_EQUAL_INT(AEGIS_DRIVE_BRAKE, act.mode);
    TEST_ASSERT_EQUAL_INT(0, act.duty_permille);
}

static void test_tracking_trips_on_second_tick(void)
{
    aegis_ctx_t ctx;
    aegis_outputs_t out;
    uint64_t now = 0u;

    arm_running(&ctx, &now);
    out = step_traj(&ctx, &now, 0, 0.f, 1.0f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx.sm.state);
    TEST_ASSERT_TRUE(out.act.duty_permille != 0);

    out = step_traj(&ctx, &now, 0, 0.f, 1.0f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_TRACKING, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
    TEST_ASSERT_EQUAL_INT(AEGIS_DRIVE_BRAKE, out.act.mode);
}

static void test_tracking_debounce_resets(void)
{
    aegis_ctx_t ctx;
    uint64_t now = 0u;

    arm_running(&ctx, &now);
    (void)step_traj(&ctx, &now, 0, 0.f, 1.0f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx.sm.state);
    (void)step_traj(&ctx, &now, 0, 0.f, 0.f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx.sm.state);
    (void)step_traj(&ctx, &now, 0, 0.f, 1.0f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx.sm.state);
    (void)step_traj(&ctx, &now, 0, 0.f, 1.0f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_TRACKING, ctx.first_fault);
}

static void test_velocity_trips(void)
{
    aegis_ctx_t ctx;
    aegis_outputs_t out;
    uint64_t now = 0u;

    arm_running(&ctx, &now);
    out = step_traj(&ctx, &now, 20, 20.f, 0.f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_VELOCITY, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void test_heartbeat_trips_after_timeout(void)
{
    aegis_ctx_t ctx;
    aegis_outputs_t out;
    uint64_t now = 0u;
    uint32_t i;

    arm_running(&ctx, &now);
    for (i = 0u; i < 10u; i++) {
        out = step_traj(&ctx, &now, 0, 0.f, 0.f, false);
        TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx.sm.state);
        TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
    }
    out = step_traj(&ctx, &now, 0, 0.f, 0.f, false);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_HEARTBEAT, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void test_disagree_trips_on_fourth_tick(void)
{
    aegis_ctx_t ctx;
    aegis_outputs_t out;
    uint64_t now = 0u;
    uint32_t i;

    arm_running(&ctx, &now);
    for (i = 0u; i < (AEGIS_DISAGREE_TICKS - 1u); i++) {
        out = step_traj(&ctx, &now, 0, 6.f, 0.f, true);
        TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx.sm.state);
    }
    out = step_traj(&ctx, &now, 0, 6.f, 0.f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_DISAGREE, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void test_stall_trips(void)
{
    aegis_ctx_t ctx;
    aegis_outputs_t out;
    uint64_t now = 0u;
    uint32_t i;

    arm_running(&ctx, &now);
    for (i = 0u; i < AEGIS_STALL_TICKS; i++) {
        out = step_traj(&ctx, &now, 0, 0.f, 0.15f, true);
        TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, ctx.sm.state);
        TEST_ASSERT_TRUE(out.act.duty_permille != 0);
    }
    out = step_traj(&ctx, &now, 0, 0.f, 0.15f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_STALL, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void test_nan_trips_numeric(void)
{
    aegis_ctx_t ctx;
    aegis_outputs_t out;
    uint64_t now = 0u;

    arm_running(&ctx, &now);
    out = step_traj(&ctx, &now, 0, nanf(""), 0.f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_NUMERIC, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void test_latch_and_plain_reset(void)
{
    aegis_ctx_t ctx;
    aegis_inputs_t in;
    aegis_outputs_t out;
    uint64_t now = 0u;
    uint32_t i;

    arm_running(&ctx, &now);
    (void)step_traj(&ctx, &now, 0, 0.f, 1.0f, true);
    (void)step_traj(&ctx, &now, 0, 0.f, 1.0f, true);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);

    for (i = 0u; i < 4u; i++) {
        clear_in(&in, now, 0, 0.f);
        in.hb_valid = true;
        in.cmd = AEGIS_CMD_TRAJ;
        in.ref_pos_rad = 0.f;
        aegis_step(&ctx, &in, &out);
        now += AEGIS_PERIOD_US;
        TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
        TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
    }

    clear_in(&in, now, 0, 0.f);
    in.hb_valid = true;
    in.cmd = AEGIS_CMD_RESET;
    aegis_step(&ctx, &in, &out);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_NONE, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void test_arm_requires_heartbeat(void)
{
    aegis_ctx_t ctx;
    aegis_inputs_t in;
    aegis_outputs_t out;

    aegis_init(&ctx);
    clear_in(&in, 0u, 0, 0.f);
    aegis_step(&ctx, &in, &out);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, ctx.sm.state);

    clear_in(&in, AEGIS_PERIOD_US, 0, 0.f);
    in.cmd = AEGIS_CMD_ARM;
    aegis_step(&ctx, &in, &out);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
}

static void test_redundant_mismatch_trips_internal(void)
{
    aegis_ctx_t ctx;
    aegis_inputs_t in;
    aegis_outputs_t out;

    aegis_init(&ctx);
    ctx.sm.state_inv = 0u;
    clear_in(&in, 0u, 0, 0.f);
    in.hb_valid = true;
    aegis_step(&ctx, &in, &out);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, ctx.sm.state);
    TEST_ASSERT_EQUAL_INT(AEGIS_FAULT_INTERNAL, ctx.first_fault);
    TEST_ASSERT_EQUAL_INT(0, out.act.duty_permille);
    TEST_ASSERT_TRUE(aegis_sm_verify(&ctx.sm));
}

static void test_state_machine_table(void)
{
    aegis_sm_t sm;

    aegis_sm_init(&sm);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_INIT, sm.state);
    TEST_ASSERT_TRUE(aegis_sm_verify(&sm));
    aegis_sm_apply(&sm, AEGIS_EV_SELFTEST_OK);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, sm.state);
    aegis_sm_apply(&sm, AEGIS_EV_ARM);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, sm.state);
    aegis_sm_apply(&sm, AEGIS_EV_STOP);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, sm.state);
    aegis_sm_apply(&sm, AEGIS_EV_ARM);
    aegis_sm_apply(&sm, AEGIS_EV_FAULT);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, sm.state);
    aegis_sm_apply(&sm, AEGIS_EV_ARM);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, sm.state);
    aegis_sm_apply(&sm, AEGIS_EV_RESET);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_ARMED, sm.state);

    sm.state = (aegis_state_t)99;
    sm.state_inv = ~((uint32_t)sm.state);
    TEST_ASSERT_TRUE(aegis_sm_verify(&sm));
    aegis_sm_apply(&sm, AEGIS_EV_NONE);
    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_TRIPPED, sm.state);
}

static void test_arbiter_property(void)
{
    static const aegis_state_t states[4] = {AEGIS_STATE_INIT, AEGIS_STATE_ARMED, AEGIS_STATE_RUNNING,
                                            AEGIS_STATE_TRIPPED};
    static const uint32_t masks[4] = {0u, 1u, 0x20u, 0xffffffffu};
    float duties[7];
    uint32_t si;
    uint32_t mi;
    uint32_t di;
    uint32_t n;
    uint32_t rng;
    aegis_actuation_t act;

    duties[0] = -2.f;
    duties[1] = -0.4f;
    duties[2] = 0.f;
    duties[3] = 0.4f;
    duties[4] = 2.f;
    duties[5] = nanf("");
    duties[6] = INFINITY;

    act = aegis_arbiter_decide(AEGIS_STATE_RUNNING, 0u, 0.5f);
    TEST_ASSERT_EQUAL_INT(500, act.duty_permille);
    TEST_ASSERT_EQUAL_INT(AEGIS_DRIVE_FWD, act.mode);
    act = aegis_arbiter_decide(AEGIS_STATE_RUNNING, 0u, -0.4f);
    TEST_ASSERT_EQUAL_INT(-400, act.duty_permille);
    TEST_ASSERT_EQUAL_INT(AEGIS_DRIVE_REV, act.mode);
    act = aegis_arbiter_decide(AEGIS_STATE_RUNNING, 0u, 2.f);
    TEST_ASSERT_EQUAL_INT(950, act.duty_permille);
    act = aegis_arbiter_decide(AEGIS_STATE_RUNNING, 1u, 0.5f);
    TEST_ASSERT_EQUAL_INT(0, act.duty_permille);
    act = aegis_arbiter_decide(AEGIS_STATE_TRIPPED, 0u, 0.5f);
    TEST_ASSERT_EQUAL_INT(0, act.duty_permille);
    act = aegis_arbiter_decide(AEGIS_STATE_RUNNING, 0u, nanf(""));
    TEST_ASSERT_EQUAL_INT(0, act.duty_permille);

    for (si = 0u; si < 4u; si++) {
        for (mi = 0u; mi < 4u; mi++) {
            for (di = 0u; di < 7u; di++) {
                act = aegis_arbiter_decide(states[si], masks[mi], duties[di]);
                if (act.duty_permille != 0) {
                    TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, states[si]);
                    TEST_ASSERT_EQUAL_UINT32(0u, masks[mi]);
                }
            }
        }
    }

    rng = 1u;
    for (n = 0u; n < 10000u; n++) {
        aegis_state_t state;
        uint32_t mask;
        float duty;

        rng = (rng * 1664525u) + 1013904223u;
        state = states[rng % 4u];
        mask = ((rng & 0x100u) != 0u) ? (rng & 0x7fu) : 0u;
        if ((rng & 0x3u) == 0u) {
            duty = nanf("");
        } else {
            duty = ((float)(int32_t)(rng % 2001u) - 1000.f) * 0.002f;
        }
        act = aegis_arbiter_decide(state, mask, duty);
        if (act.duty_permille != 0) {
            TEST_ASSERT_EQUAL_INT(AEGIS_STATE_RUNNING, state);
            TEST_ASSERT_EQUAL_UINT32(0u, mask);
        }
    }
}

static void test_spsc_order_and_drop(void)
{
    aegis_spsc_t q;
    uint32_t storage[8];
    uint32_t i;
    uint32_t got;

    TEST_ASSERT_EQUAL_INT(AEGIS_E_ARG, aegis_spsc_init(NULL, storage, 8u, sizeof(uint32_t)));
    TEST_ASSERT_EQUAL_INT(AEGIS_E_RANGE, aegis_spsc_init(&q, storage, 3u, sizeof(uint32_t)));
    TEST_ASSERT_EQUAL_INT(AEGIS_OK, aegis_spsc_init(&q, storage, 8u, sizeof(uint32_t)));

    for (i = 0u; i < 8u; i++) {
        TEST_ASSERT_TRUE(aegis_spsc_push(&q, &i));
    }
    i = 99u;
    TEST_ASSERT_FALSE(aegis_spsc_push(&q, &i));
    TEST_ASSERT_EQUAL_UINT32(1u, q.dropped);
    for (i = 0u; i < 8u; i++) {
        TEST_ASSERT_TRUE(aegis_spsc_pop(&q, &got));
        TEST_ASSERT_EQUAL_UINT32(i, got);
    }
    TEST_ASSERT_FALSE(aegis_spsc_pop(&q, &got));
}

typedef struct {
    uint32_t seq;
} spsc_item_t;

typedef struct {
    aegis_spsc_t *q;
    uint32_t produced;
    uint32_t received;
    int bad_order;
} spsc_thread_ctx_t;

static _Atomic uint32_t g_producer_done;

static void *spsc_producer(void *arg)
{
    spsc_thread_ctx_t *ctx = arg;
    uint32_t i;

    for (i = 0u; i < ctx->produced; i++) {
        spsc_item_t item;
        item.seq = i;
        (void)aegis_spsc_push(ctx->q, &item);
    }
    atomic_store_explicit(&g_producer_done, 1u, memory_order_release);
    return NULL;
}

static void *spsc_consumer(void *arg)
{
    spsc_thread_ctx_t *ctx = arg;
    uint32_t last = 0u;
    bool have = false;

    for (;;) {
        spsc_item_t item;
        if (aegis_spsc_pop(ctx->q, &item)) {
            if (have && (item.seq <= last)) {
                ctx->bad_order = 1;
            }
            last = item.seq;
            have = true;
            ctx->received += 1u;
            continue;
        }
        if (atomic_load_explicit(&g_producer_done, memory_order_acquire) != 0u) {
            if (!aegis_spsc_pop(ctx->q, &item)) {
                break;
            }
            if (have && (item.seq <= last)) {
                ctx->bad_order = 1;
            }
            last = item.seq;
            have = true;
            ctx->received += 1u;
            continue;
        }
        sched_yield();
    }
    return NULL;
}

static void test_spsc_threads(void)
{
    enum { N = 100000, CAP = 64 };
    aegis_spsc_t q;
    spsc_item_t storage[CAP];
    spsc_thread_ctx_t ctx;
    pthread_t prod;
    pthread_t cons;

    TEST_ASSERT_EQUAL_INT(AEGIS_OK, aegis_spsc_init(&q, storage, (uint32_t)CAP, sizeof(spsc_item_t)));
    memset(&ctx, 0, sizeof(ctx));
    ctx.q = &q;
    ctx.produced = (uint32_t)N;
    atomic_store_explicit(&g_producer_done, 0u, memory_order_relaxed);
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&cons, NULL, spsc_consumer, &ctx));
    TEST_ASSERT_EQUAL_INT(0, pthread_create(&prod, NULL, spsc_producer, &ctx));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(prod, NULL));
    TEST_ASSERT_EQUAL_INT(0, pthread_join(cons, NULL));
    TEST_ASSERT_EQUAL_INT(0, ctx.bad_order);
    TEST_ASSERT_TRUE(ctx.received > 0u);
    TEST_ASSERT_EQUAL_UINT32(ctx.produced, ctx.received + q.dropped);
}

static void test_lockstep_campaign(void)
{
    uint32_t i;

    for (i = 0u; i < sim_scenario_count(); i++) {
        const sim_scenario_t *sc = sim_scenario_at(i);
        sim_summary_t sum;

        sim_run(sc, &sum, NULL, 0u);
        TEST_ASSERT_FALSE(sum.duty_outside_running);
        TEST_ASSERT_FALSE(sum.motion_after_trip);
        TEST_ASSERT_EQUAL_UINT32(sc->duration_ticks, sum.spsc_pushed + sum.spsc_dropped);
        TEST_ASSERT_TRUE(sum.spsc_dropped > 0u);
        if (sc->expect_fault == AEGIS_FAULT_NONE) {
            TEST_ASSERT_FALSE(sum.tripped);
            TEST_ASSERT_TRUE(sum.nonzero_duty_ticks > 0u);
        } else {
            TEST_ASSERT_TRUE(sum.tripped);
            TEST_ASSERT_TRUE(sum.trip_tick >= sc->onset_tick);
            TEST_ASSERT_EQUAL_INT(sc->expect_fault, sum.fault);
        }
    }
}

static void test_deterministic_replay(void)
{
    const sim_scenario_t *nominal = sim_scenario_at(0u);
    const sim_scenario_t *other = sim_scenario_at(1u);
    aegis_telem_t a[SIM_TRACE_MAX];
    aegis_telem_t b[SIM_TRACE_MAX];
    aegis_telem_t c[SIM_TRACE_MAX];
    sim_summary_t sa;
    sim_summary_t sb;
    sim_summary_t sc;

    TEST_ASSERT_TRUE(nominal->duration_ticks <= SIM_TRACE_MAX);
    sim_run(nominal, &sa, a, SIM_TRACE_MAX);
    sim_run(nominal, &sb, b, SIM_TRACE_MAX);
    sim_run(other, &sc, c, SIM_TRACE_MAX);
    TEST_ASSERT_EQUAL_INT(0, memcmp(a, b, sizeof(aegis_telem_t) * nominal->duration_ticks));
    TEST_ASSERT_TRUE(memcmp(a, c, sizeof(aegis_telem_t) * nominal->duration_ticks) != 0);
}

static void test_recorded_inputs_replay_open_loop(void)
{
    static aegis_inputs_t in[SIM_TRACE_MAX];
    static aegis_telem_t closed[SIM_TRACE_MAX];
    uint32_t i;

    for (i = 0u; i < sim_scenario_count(); i++) {
        const sim_scenario_t *sc = sim_scenario_at(i);
        aegis_ctx_t ctx;
        sim_summary_t sum;
        uint32_t n;
        uint32_t t;

        n = sim_record_inputs(sc, in, SIM_TRACE_MAX);
        sim_run(sc, &sum, closed, SIM_TRACE_MAX);
        TEST_ASSERT_EQUAL_UINT32(sum.n_ticks, n);
        aegis_init(&ctx);
        for (t = 0u; t < n; t++) {
            aegis_outputs_t out;
            aegis_step(&ctx, &in[t], &out);
            TEST_ASSERT_EQUAL_INT(0, memcmp(&out.telem, &closed[t], sizeof(out.telem)));
        }
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_zero_actuation_is_safe);
    RUN_TEST(test_tracking_trips_on_second_tick);
    RUN_TEST(test_tracking_debounce_resets);
    RUN_TEST(test_velocity_trips);
    RUN_TEST(test_heartbeat_trips_after_timeout);
    RUN_TEST(test_disagree_trips_on_fourth_tick);
    RUN_TEST(test_stall_trips);
    RUN_TEST(test_nan_trips_numeric);
    RUN_TEST(test_latch_and_plain_reset);
    RUN_TEST(test_arm_requires_heartbeat);
    RUN_TEST(test_redundant_mismatch_trips_internal);
    RUN_TEST(test_state_machine_table);
    RUN_TEST(test_arbiter_property);
    RUN_TEST(test_spsc_order_and_drop);
    RUN_TEST(test_spsc_threads);
    RUN_TEST(test_lockstep_campaign);
    RUN_TEST(test_deterministic_replay);
    RUN_TEST(test_recorded_inputs_replay_open_loop);
    return UNITY_END();
}
