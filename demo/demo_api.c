#include "demo_api.h"

#include "aegis/safety.h"
#include "aegis/spsc.h"
#include "aegis_config.h"
#include "sim/plant.h"

#include <math.h>
#include <string.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define EXPORT
#endif

#define RAMP_RAD_S 1.0f
#define GYRO_BIAS_RAD_S 5.0f
#define SPIN_RAD_S 25.0f
#define JUMP_RAD 1.0f

static aegis_ctx_t ctx;
static sim_plant_t plant;
static aegis_spsc_t ring;
static aegis_telem_t ring_storage[AEGIS_SPSC_CAP];
static aegis_telem_t sample;

static float applied_duty;
static float target;
static float ref;
static int pending_cmd;
static int jump_pending;
static int faults[DEMO_FAULT_COUNT];
static uint32_t tick;

EXPORT void demo_reset(void)
{
    aegis_status_t st;

    aegis_init(&ctx);
    sim_plant_init(&plant);
    st = aegis_spsc_init(&ring, ring_storage, AEGIS_SPSC_CAP, (uint32_t)sizeof(aegis_telem_t));
    (void)st;
    memset(&sample, 0, sizeof(sample));
    memset(faults, 0, sizeof(faults));
    applied_duty = 0.0f;
    target = 0.0f;
    ref = 0.0f;
    pending_cmd = 0;
    jump_pending = 0;
    tick = 0;
}

EXPORT void demo_send(int cmd)
{
    pending_cmd = cmd;
}

EXPORT void demo_set_target(float target_rad)
{
    target = target_rad;
}

EXPORT void demo_set_fault(int fault, int on)
{
    if (fault < 0 || fault >= DEMO_FAULT_COUNT) {
        return;
    }
    faults[fault] = on ? 1 : 0;
}

EXPORT void demo_jump_ref(void)
{
    jump_pending = 1;
}

/* the reference moves toward the slider target at a fixed speed so a normal move does not trip */
static float move_ref(void)
{
    float step = RAMP_RAD_S * AEGIS_DT_S;

    if (jump_pending) {
        jump_pending = 0;
        ref = ref + JUMP_RAD;
        return 0.0f;
    }
    if (target > ref + step) {
        ref = ref + step;
        return RAMP_RAD_S;
    }
    if (target < ref - step) {
        ref = ref - step;
        return -RAMP_RAD_S;
    }
    ref = target;
    return 0.0f;
}

EXPORT void demo_tick(void)
{
    aegis_inputs_t in;
    aegis_outputs_t out;

    if (faults[DEMO_FAULT_JAM]) {
        sim_plant_freeze(&plant);
    } else if (faults[DEMO_FAULT_SPIN]) {
        sim_plant_force_omega(&plant, SPIN_RAD_S, AEGIS_DT_S);
    } else {
        sim_plant_step(&plant, applied_duty, AEGIS_DT_S);
    }

    memset(&in, 0, sizeof(in));
    in.now_us = (uint64_t)tick * AEGIS_PERIOD_US;
    in.enc_count = plant.enc_count;
    in.gyro_rad_s = plant.omega_rad_s;
    if (faults[DEMO_FAULT_GYRO_BIAS]) {
        in.gyro_rad_s = in.gyro_rad_s + GYRO_BIAS_RAD_S;
    }
    if (faults[DEMO_FAULT_GYRO_NAN]) {
        in.gyro_rad_s = NAN;
    }
    in.hb_valid = !faults[DEMO_FAULT_HEARTBEAT];

    if (pending_cmd == DEMO_CMD_ARM) {
        in.cmd = AEGIS_CMD_ARM;
    } else if (pending_cmd == DEMO_CMD_STOP) {
        in.cmd = AEGIS_CMD_STOP;
    } else if (pending_cmd == DEMO_CMD_RESET) {
        in.cmd = AEGIS_CMD_RESET;
    } else {
        in.cmd = AEGIS_CMD_TRAJ;
        in.ref_vel_rad_s = move_ref();
        in.ref_pos_rad = ref;
    }
    pending_cmd = 0;

    aegis_step(&ctx, &in, &out);
    applied_duty = (float)out.act.duty_permille / 1000.0f;

    /* while not running, keep the reference on the shaft so the next move starts from there */
    if (out.telem.state != AEGIS_STATE_RUNNING) {
        ref = out.telem.theta_rad;
        jump_pending = 0;
    }

    (void)aegis_spsc_push(&ring, &out.telem);
    tick = tick + 1;
}

EXPORT int demo_pop(void)
{
    if (aegis_spsc_pop(&ring, &sample)) {
        return 1;
    }
    return 0;
}

EXPORT int demo_tick_count(void)
{
    return (int)sample.tick;
}

EXPORT float demo_angle(void)
{
    return sample.theta_rad;
}

EXPORT float demo_ref(void)
{
    return sample.ref_rad;
}

EXPORT float demo_speed(void)
{
    return sample.omega_rad_s;
}

EXPORT int demo_duty(void)
{
    return sample.duty_permille;
}

EXPORT int demo_state(void)
{
    return sample.state;
}

EXPORT int demo_fault(void)
{
    return sample.fault;
}

EXPORT int demo_dropped(void)
{
    return (int)ring.dropped;
}
