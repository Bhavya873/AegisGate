#include "demo_api.h"

#include "sim/arm.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define EXPORT
#endif

static sim_arm_host_t host;
static sim_arm_gated_t gated;
static sim_arm_nogate_t nogate;
static uint32_t tick;
static int fault;
static uint32_t fault_tick;

static int bad_joint(int joint)
{
    return joint < 0 || joint >= AEGIS_ARM_JOINTS;
}

EXPORT void demo_arm_reset(void)
{
    sim_arm_host_init(&host);
    sim_arm_gated_init(&gated);
    sim_arm_nogate_init(&nogate);
    tick = 0;
    fault = SIM_ARM_FAULT_NONE;
    fault_tick = 0;
}

/* only one fault per run, press Start over for another */
EXPORT void demo_arm_break(int new_fault)
{
    if (fault != SIM_ARM_FAULT_NONE) {
        return;
    }
    if (new_fault <= SIM_ARM_FAULT_NONE || new_fault >= SIM_ARM_FAULT_COUNT) {
        return;
    }
    fault = new_fault;
    fault_tick = tick;
}

/* both arms are stepped from the same command and the same fault */
EXPORT void demo_arm_tick(void)
{
    sim_arm_cmd_t cmd;

    sim_arm_host_step(&host, tick, (sim_arm_fault_t)fault, &cmd);
    sim_arm_nogate_step(&nogate, tick, &cmd);
    sim_arm_gated_step(&gated, tick, &cmd);
    tick = tick + 1;
}

EXPORT int demo_arm_tick_count(void)
{
    return (int)tick;
}

EXPORT int demo_arm_fault(void)
{
    return fault;
}

EXPORT int demo_arm_fault_tick(void)
{
    return (int)fault_tick;
}

EXPORT float demo_arm_angle(int side, int joint)
{
    if (bad_joint(joint)) {
        return 0.0f;
    }
    if (side == DEMO_SIDE_GATED) {
        return gated.joint[joint].theta_rad;
    }
    return nogate.joint[joint].theta_rad;
}

EXPORT float demo_arm_speed(int side, int joint)
{
    if (bad_joint(joint)) {
        return 0.0f;
    }
    if (side == DEMO_SIDE_GATED) {
        return gated.joint[joint].omega_rad_s;
    }
    return nogate.joint[joint].omega_rad_s;
}

EXPORT int demo_arm_duty(int side, int joint)
{
    if (bad_joint(joint)) {
        return 0;
    }
    if (side == DEMO_SIDE_GATED) {
        return gated.telem[joint].duty_permille;
    }
    return nogate.duty_permille[joint];
}

EXPORT float demo_arm_link(int joint)
{
    if (joint == 0) {
        return AEGIS_ARM_LINK0_M;
    }
    return AEGIS_ARM_LINK1_M;
}

EXPORT int demo_arm_state(int joint)
{
    if (bad_joint(joint)) {
        return AEGIS_STATE_TRIPPED;
    }
    return gated.telem[joint].state;
}

EXPORT int demo_arm_tripped(void)
{
    return gated.tripped ? 1 : 0;
}

EXPORT int demo_arm_trip_fault(void)
{
    return gated.trip_fault;
}

EXPORT int demo_arm_trip_tick(void)
{
    return (int)gated.trip_tick;
}
