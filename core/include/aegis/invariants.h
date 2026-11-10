#ifndef AEGIS_INVARIANTS_H
#define AEGIS_INVARIANTS_H

#include "aegis/types.h"

#include "aegis_config.h"

typedef struct {
    uint8_t n_tracking;
    uint8_t n_velocity;
    uint8_t n_heartbeat;
    uint8_t n_disagree;
    int32_t stall_deltas[AEGIS_STALL_TICKS];
    uint8_t stall_n;
    uint8_t stall_i;
    int32_t prev_count;
    bool enc_primed;
} aegis_inv_t;

typedef struct {
    float theta_hat_rad;
    float omega_hat_rad_s;
    float omega_enc_rad_s;
    float omega_gyro_rad_s;
    float ref_rad;
    float applied_duty;
    int32_t enc_count;
    uint64_t now_us;
    uint64_t last_hb_us;
    bool hb_seen;
    bool in_running;
} aegis_inv_inputs_t;

void aegis_inv_init(aegis_inv_t *inv);
uint32_t aegis_inv_eval(aegis_inv_t *inv, const aegis_inv_inputs_t *in);

#endif
