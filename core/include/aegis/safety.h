#ifndef AEGIS_SAFETY_H
#define AEGIS_SAFETY_H

#include "aegis/estimator.h"
#include "aegis/invariants.h"
#include "aegis/pid.h"
#include "aegis/state_machine.h"
#include "aegis/types.h"

typedef struct {
    uint64_t now_us;
    int32_t enc_count;
    float gyro_rad_s;
    bool hb_valid;
    aegis_cmd_type_t cmd;
    float ref_pos_rad;
    float ref_vel_rad_s;
} aegis_inputs_t;

typedef struct {
    aegis_actuation_t act;
    aegis_telem_t telem;
} aegis_outputs_t;

typedef struct {
    aegis_sm_t sm;
    aegis_inv_t inv;
    aegis_est_t est;
    aegis_pid_t pid;
    uint32_t fault_mask;
    aegis_fault_t first_fault;
    uint64_t last_hb_us;
    bool hb_seen;
    float ref_rad;
    float ref_vel;
    float applied_duty;
    uint32_t tick;
} aegis_ctx_t;

void aegis_init(aegis_ctx_t *ctx);
void aegis_step(aegis_ctx_t *ctx, const aegis_inputs_t *in, aegis_outputs_t *out);

#endif
