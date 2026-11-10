#ifndef AEGIS_ESTIMATOR_H
#define AEGIS_ESTIMATOR_H

#include "aegis/types.h"

typedef struct {
    int32_t prev_count;
    float theta_rad;
    float omega_hat;
    float omega_enc;
    float omega_gyro;
    float gyro_prev;
    bool primed;
} aegis_est_t;

void aegis_est_init(aegis_est_t *est);
void aegis_est_update(aegis_est_t *est, int32_t count, float gyro_rad_s);

#endif
