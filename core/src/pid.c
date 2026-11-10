#include "aegis/pid.h"

#include "aegis_config.h"

void aegis_pid_init(aegis_pid_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integ = 0.f;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

float aegis_pid_step(aegis_pid_t *pid, float error_rad, float omega_hat, float omega_ref)
{
    float u_p;
    float u_d;
    float u;
    bool sat_high;
    bool sat_low;
    bool freeze;

    if (pid == NULL) {
        return 0.f;
    }

    u_p = AEGIS_PID_KP * error_rad;
    u_d = AEGIS_PID_KD * (omega_ref - omega_hat);
    u = u_p + pid->integ + u_d;
    sat_high = u > AEGIS_DUTY_MAX;
    sat_low = u < -AEGIS_DUTY_MAX;
    freeze = (sat_high && (error_rad > 0.f)) || (sat_low && (error_rad < 0.f));
    if (!freeze) {
        pid->integ += AEGIS_PID_KI * error_rad * AEGIS_DT_S;
        pid->integ = clampf(pid->integ, -AEGIS_PID_I_CLAMP, AEGIS_PID_I_CLAMP);
    }
    return clampf(u, -AEGIS_DUTY_MAX, AEGIS_DUTY_MAX);
}
