#include "aegis/arbiter.h"

#include "aegis_config.h"

aegis_actuation_t aegis_arbiter_decide(aegis_state_t state, uint32_t fault_mask, float pid_duty)
{
    aegis_actuation_t act;
    float duty;
    float scaled;
    int32_t milli;

    act.mode = AEGIS_DRIVE_BRAKE;
    act.duty_permille = 0;

    if (state != AEGIS_STATE_RUNNING) {
        return act;
    }
    if (fault_mask != 0u) {
        return act;
    }
    if (!aegis_is_finite(pid_duty)) {
        return act;
    }

    duty = pid_duty;
    if (duty > AEGIS_DUTY_MAX) {
        duty = AEGIS_DUTY_MAX;
    } else if (duty < -AEGIS_DUTY_MAX) {
        duty = -AEGIS_DUTY_MAX;
    }

    scaled = duty * 1000.f;
    if (scaled > 1000.f) {
        scaled = 1000.f;
    } else if (scaled < -1000.f) {
        scaled = -1000.f;
    }
    milli = (int32_t)scaled;
    if (milli == 0) {
        return act;
    }

    act.duty_permille = (int16_t)milli;
    if (milli > 0) {
        act.mode = AEGIS_DRIVE_FWD;
    } else {
        act.mode = AEGIS_DRIVE_REV;
    }
    return act;
}
