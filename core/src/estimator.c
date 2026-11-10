#include "aegis/estimator.h"

#include "aegis_config.h"

void aegis_est_init(aegis_est_t *est)
{
    if (est == NULL) {
        return;
    }
    est->prev_count = 0;
    est->theta_rad = 0.f;
    est->omega_hat = 0.f;
    est->omega_enc = 0.f;
    est->omega_gyro = 0.f;
    est->gyro_prev = 0.f;
    est->primed = false;
}

void aegis_est_update(aegis_est_t *est, int32_t count, float gyro_rad_s)
{
    float k_c;
    float alpha;
    int32_t dc;
    float omega_enc;
    float d_g;

    if (est == NULL) {
        return;
    }
    k_c = AEGIS_K_C_RAD;
    if (!est->primed) {
        est->prev_count = count;
        est->theta_rad = (float)count * k_c;
        est->omega_hat = 0.f;
        est->omega_enc = 0.f;
        est->omega_gyro = gyro_rad_s;
        est->gyro_prev = gyro_rad_s;
        est->primed = true;
        return;
    }

    dc = count - est->prev_count;
    omega_enc = ((float)dc * k_c) / AEGIS_DT_S;
    d_g = gyro_rad_s - est->gyro_prev;
    alpha = AEGIS_COMP_TAU_S / (AEGIS_COMP_TAU_S + AEGIS_DT_S);
    est->omega_hat = (alpha * (est->omega_hat + d_g)) + ((1.f - alpha) * omega_enc);
    est->omega_enc = omega_enc;
    est->omega_gyro = gyro_rad_s;
    est->gyro_prev = gyro_rad_s;
    est->prev_count = count;
    est->theta_rad = (float)count * k_c;
}
