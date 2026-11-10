#include "aegis/invariants.h"

#include <string.h>

static bool debounce(uint8_t *n, bool bad, uint8_t limit)
{
    if (bad) {
        if (*n < 255u) {
            *n = (uint8_t)(*n + 1u);
        }
    } else {
        *n = 0u;
    }
    return *n >= limit;
}

static float absf(float x)
{
    if (x < 0.f) {
        return -x;
    }
    return x;
}

static void stall_clear(aegis_inv_t *inv)
{
    inv->stall_n = 0u;
    inv->stall_i = 0u;
}

void aegis_inv_init(aegis_inv_t *inv)
{
    if (inv == NULL) {
        return;
    }
    memset(inv, 0, sizeof(*inv));
}

uint32_t aegis_inv_eval(aegis_inv_t *inv, const aegis_inv_inputs_t *in)
{
    uint32_t mask = 0u;
    int32_t delta = 0;
    int32_t sum;
    uint8_t k;
    float duty;
    float err;
    float speed;
    float diff;
    float peak;
    float gyro_abs;
    float thresh;
    bool hb_bad;

    if (inv == NULL || in == NULL) {
        return AEGIS_BIT_INTERNAL;
    }

    /* every check uses a strict compare, so exactly at the threshold does not trip */
    if (in->in_running) {
        err = absf(in->ref_rad - in->theta_hat_rad);
        if (debounce(&inv->n_tracking, err > AEGIS_TRACK_EPS_RAD, (uint8_t)AEGIS_TRACK_TICKS)) {
            mask |= AEGIS_BIT_TRACKING;
        }
    } else {
        inv->n_tracking = 0u;
    }

    speed = absf(in->omega_hat_rad_s);
    if (debounce(&inv->n_velocity, speed > AEGIS_V_MAX_RAD_S, (uint8_t)AEGIS_V_TICKS)) {
        mask |= AEGIS_BIT_VELOCITY;
    }

    hb_bad = false;
    if (in->in_running && in->hb_seen) {
        if (in->now_us < in->last_hb_us) {
            hb_bad = true;
        } else if (in->now_us - in->last_hb_us > AEGIS_HB_TIMEOUT_US) {
            hb_bad = true;
        }
    }
    if (debounce(&inv->n_heartbeat, hb_bad, (uint8_t)AEGIS_HB_TICKS)) {
        mask |= AEGIS_BIT_HEARTBEAT;
    }

    diff = absf(in->omega_enc_rad_s - in->omega_gyro_rad_s);
    peak = absf(in->omega_enc_rad_s);
    gyro_abs = absf(in->omega_gyro_rad_s);
    if (gyro_abs > peak) {
        peak = gyro_abs;
    }
    thresh = AEGIS_DISAGREE_FRAC * peak;
    if (thresh < AEGIS_OMEGA_DB_RAD_S) {
        thresh = AEGIS_OMEGA_DB_RAD_S;
    }
    if (debounce(&inv->n_disagree, diff > thresh, (uint8_t)AEGIS_DISAGREE_TICKS)) {
        mask |= AEGIS_BIT_DISAGREE;
    }

    if (inv->enc_primed) {
        delta = in->enc_count - inv->prev_count;
    } else {
        inv->enc_primed = true;
    }
    inv->prev_count = in->enc_count;

    duty = absf(in->applied_duty);
    if (duty >= AEGIS_STALL_DUTY) {
        inv->stall_deltas[inv->stall_i] = delta;
        inv->stall_i = (uint8_t)((inv->stall_i + 1u) % AEGIS_STALL_TICKS);
        if (inv->stall_n < AEGIS_STALL_TICKS) {
            inv->stall_n = (uint8_t)(inv->stall_n + 1u);
        }
        if (inv->stall_n >= AEGIS_STALL_TICKS) {
            sum = 0;
            for (k = 0u; k < AEGIS_STALL_TICKS; k++) {
                sum += inv->stall_deltas[k];
            }
            if (sum < 0) {
                sum = -sum;
            }
            if (sum <= AEGIS_STALL_COUNT_SUM) {
                mask |= AEGIS_BIT_STALL;
            }
        }
    } else {
        stall_clear(inv);
    }

    return mask;
}
