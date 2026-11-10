#include "aegis/safety.h"

#include "aegis/arbiter.h"
#include "aegis_config.h"

#include <string.h>

static bool hb_fresh(const aegis_ctx_t *ctx, uint64_t now_us)
{
    if (!ctx->hb_seen) {
        return false;
    }
    if (now_us < ctx->last_hb_us) {
        return false;
    }
    return now_us - ctx->last_hb_us <= AEGIS_HB_TIMEOUT_US;
}

static void latch_fault(aegis_ctx_t *ctx, uint32_t mask)
{
    if (mask == 0u) {
        return;
    }
    if (ctx->sm.state == AEGIS_STATE_TRIPPED) {
        return;
    }
    if (ctx->first_fault == AEGIS_FAULT_NONE) {
        ctx->first_fault = aegis_fault_from_mask(mask);
    }
    ctx->fault_mask |= mask;
    aegis_sm_apply(&ctx->sm, AEGIS_EV_FAULT);
}

void aegis_init(aegis_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    aegis_sm_init(&ctx->sm);
    aegis_inv_init(&ctx->inv);
    aegis_est_init(&ctx->est);
    aegis_pid_init(&ctx->pid);
}

void aegis_step(aegis_ctx_t *ctx, const aegis_inputs_t *in, aegis_outputs_t *out)
{
    aegis_actuation_t safe;
    bool gyro_ok;
    bool traj_ok;
    bool bad_cmd;
    bool want_arm;
    bool want_stop;
    bool want_reset;
    uint32_t live;
    float pid_duty;
    float err;
    aegis_inv_inputs_t ii;

    safe.mode = AEGIS_DRIVE_BRAKE;
    safe.duty_permille = 0;
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        out->act = safe;
    }
    if (ctx == NULL || in == NULL || out == NULL) {
        return;
    }

    if (!aegis_sm_verify(&ctx->sm)) {
        aegis_sm_set(&ctx->sm, AEGIS_STATE_TRIPPED);
        ctx->fault_mask |= AEGIS_BIT_INTERNAL;
        if (ctx->first_fault == AEGIS_FAULT_NONE) {
            ctx->first_fault = AEGIS_FAULT_INTERNAL;
        }
    }

    gyro_ok = aegis_is_finite(in->gyro_rad_s);
    traj_ok = true;
    bad_cmd = false;
    want_arm = false;
    want_stop = false;
    want_reset = false;

    switch (in->cmd) {
    case AEGIS_CMD_NONE:
    case AEGIS_CMD_HEARTBEAT:
        break;
    case AEGIS_CMD_TRAJ:
        traj_ok = aegis_is_finite(in->ref_pos_rad) && aegis_is_finite(in->ref_vel_rad_s);
        if (traj_ok) {
            ctx->ref_rad = in->ref_pos_rad;
            ctx->ref_vel = in->ref_vel_rad_s;
        }
        break;
    case AEGIS_CMD_ARM:
        want_arm = true;
        break;
    case AEGIS_CMD_STOP:
        want_stop = true;
        break;
    case AEGIS_CMD_RESET:
        want_reset = true;
        break;
    default:
        bad_cmd = true;
        break;
    }

    if (in->hb_valid && gyro_ok) {
        ctx->last_hb_us = in->now_us;
        ctx->hb_seen = true;
    }

    if (gyro_ok) {
        aegis_est_update(&ctx->est, in->enc_count, in->gyro_rad_s);
    }

    live = 0u;
    if (!gyro_ok || !traj_ok) {
        live |= AEGIS_BIT_NUMERIC;
    }
    if (bad_cmd) {
        live |= AEGIS_BIT_INTERNAL;
    }
    if (ctx->sm.state != AEGIS_STATE_INIT && gyro_ok) {
        memset(&ii, 0, sizeof(ii));
        ii.theta_hat_rad = ctx->est.theta_rad;
        ii.omega_hat_rad_s = ctx->est.omega_hat;
        ii.omega_enc_rad_s = ctx->est.omega_enc;
        ii.omega_gyro_rad_s = ctx->est.omega_gyro;
        ii.ref_rad = ctx->ref_rad;
        ii.applied_duty = ctx->applied_duty;
        ii.enc_count = in->enc_count;
        ii.now_us = in->now_us;
        ii.last_hb_us = ctx->last_hb_us;
        ii.hb_seen = ctx->hb_seen;
        ii.in_running = (ctx->sm.state == AEGIS_STATE_RUNNING);
        live |= aegis_inv_eval(&ctx->inv, &ii);
    }
    if (!aegis_is_finite(ctx->est.theta_rad) || !aegis_is_finite(ctx->est.omega_hat) ||
        !aegis_is_finite(ctx->est.omega_enc) || !aegis_is_finite(ctx->est.omega_gyro)) {
        live |= AEGIS_BIT_NUMERIC;
    }

    if (ctx->sm.state == AEGIS_STATE_INIT && live == 0u && gyro_ok) {
        aegis_sm_apply(&ctx->sm, AEGIS_EV_SELFTEST_OK);
    }

    if (live != 0u && ctx->sm.state != AEGIS_STATE_TRIPPED) {
        latch_fault(ctx, live);
    } else if (ctx->sm.state == AEGIS_STATE_TRIPPED && want_reset) {
        ctx->fault_mask = 0u;
        ctx->first_fault = AEGIS_FAULT_NONE;
        aegis_inv_init(&ctx->inv);
        aegis_pid_init(&ctx->pid);
        aegis_sm_apply(&ctx->sm, AEGIS_EV_RESET);
    } else if (ctx->sm.state == AEGIS_STATE_ARMED && want_arm && live == 0u && in->hb_valid &&
               hb_fresh(ctx, in->now_us)) {
        aegis_pid_init(&ctx->pid);
        ctx->ref_rad = ctx->est.theta_rad;
        ctx->ref_vel = 0.f;
        aegis_sm_apply(&ctx->sm, AEGIS_EV_ARM);
    } else if (want_stop &&
               (ctx->sm.state == AEGIS_STATE_RUNNING || ctx->sm.state == AEGIS_STATE_ARMED)) {
        aegis_sm_apply(&ctx->sm, AEGIS_EV_STOP);
    }

    pid_duty = 0.f;
    if (ctx->sm.state == AEGIS_STATE_RUNNING && ctx->fault_mask == 0u) {
        err = ctx->ref_rad - ctx->est.theta_rad;
        pid_duty = aegis_pid_step(&ctx->pid, err, ctx->est.omega_hat, ctx->ref_vel);
        if (!aegis_is_finite(pid_duty)) {
            latch_fault(ctx, AEGIS_BIT_NUMERIC);
            pid_duty = 0.f;
        }
    }

    out->act = aegis_arbiter_decide(ctx->sm.state, ctx->fault_mask, pid_duty);
    ctx->applied_duty = (float)out->act.duty_permille / 1000.f;

    out->telem.tick = ctx->tick;
    out->telem.theta_rad = ctx->est.theta_rad;
    out->telem.omega_rad_s = ctx->est.omega_hat;
    out->telem.ref_rad = ctx->ref_rad;
    out->telem.duty_permille = out->act.duty_permille;
    out->telem.state = (uint8_t)ctx->sm.state;
    out->telem.fault = (uint8_t)ctx->first_fault;
    ctx->tick += 1u;
}
