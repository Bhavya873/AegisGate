#include "sim/lockstep.h"

#include "aegis/spsc.h"
#include "aegis_config.h"
#include "sim/plant.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

static const sim_scenario_t k_scenarios[] = {
    {"nominal", SIM_FAULT_NONE, 0u, 500u, AEGIS_FAULT_NONE},
    {"heartbeat_drop", SIM_FAULT_HB_DROP, 300u, 500u, AEGIS_FAULT_HEARTBEAT},
    {"shaft_stall", SIM_FAULT_STALL, 80u, 500u, AEGIS_FAULT_STALL},
    {"gyro_bias", SIM_FAULT_GYRO_BIAS, 300u, 500u, AEGIS_FAULT_DISAGREE},
    {"overspeed", SIM_FAULT_OVERSPEED, 300u, 500u, AEGIS_FAULT_VELOCITY},
    {"tracking_step", SIM_FAULT_TRACKING, 300u, 500u, AEGIS_FAULT_TRACKING},
};

uint32_t sim_scenario_count(void)
{
    return (uint32_t)(sizeof(k_scenarios) / sizeof(k_scenarios[0]));
}

const sim_scenario_t *sim_scenario_at(uint32_t index)
{
    if (index >= sim_scenario_count()) {
        return NULL;
    }
    return &k_scenarios[index];
}

static void reference_at(const sim_scenario_t *scenario, uint32_t tick, const sim_plant_t *plant,
                         bool fault_on, float *ref_rad, float *ref_vel)
{
    float ref = 0.f;
    float vel = 0.f;

    if ((tick >= AEGIS_RAMP_START_TICK) && (tick < AEGIS_RAMP_END_TICK)) {
        vel = AEGIS_RAMP_VEL_RAD_S;
        ref = AEGIS_RAMP_VEL_RAD_S * ((float)(tick - AEGIS_RAMP_START_TICK) * AEGIS_DT_S);
        if (ref > AEGIS_RAMP_END_RAD) {
            ref = AEGIS_RAMP_END_RAD;
        }
    } else if (tick >= AEGIS_RAMP_END_TICK) {
        ref = AEGIS_RAMP_END_RAD;
        vel = 0.f;
    }

    if (fault_on && (scenario->kind == SIM_FAULT_TRACKING)) {
        ref = plant->theta_rad + 1.0f;
        vel = 0.f;
    } else if (fault_on && (scenario->kind == SIM_FAULT_OVERSPEED)) {
        ref = plant->theta_rad;
        vel = 25.f;
    } else if (fault_on && (scenario->kind == SIM_FAULT_STALL)) {
        ref = plant->theta_rad + 0.12f;
        vel = 0.f;
    }

    *ref_rad = ref;
    *ref_vel = vel;
}

static void run_scenario(const sim_scenario_t *scenario, sim_summary_t *summary,
                         aegis_telem_t *trace, uint32_t trace_cap, aegis_inputs_t *inputs,
                         uint32_t inputs_cap)
{
    aegis_ctx_t ctx;
    sim_plant_t plant;
    aegis_spsc_t ring;
    _Alignas(128) uint8_t storage[AEGIS_SPSC_CAP * sizeof(aegis_telem_t)];
    float applied = 0.f;
    uint32_t tick;
    bool saw_trip = false;

    if (summary == NULL) {
        return;
    }
    memset(summary, 0, sizeof(*summary));
    if (scenario == NULL) {
        return;
    }
    summary->onset_tick = scenario->onset_tick;

    aegis_init(&ctx);
    sim_plant_init(&plant);
    if (aegis_spsc_init(&ring, storage, AEGIS_SPSC_CAP, (uint32_t)sizeof(aegis_telem_t)) != AEGIS_OK) {
        return;
    }

    for (tick = 0u; tick < scenario->duration_ticks; tick++) {
        aegis_inputs_t in;
        aegis_outputs_t out;
        bool fault_on;
        float ref = 0.f;
        float ref_vel = 0.f;
        float abs_err;

        fault_on = (scenario->kind != SIM_FAULT_NONE) && (tick >= scenario->onset_tick);
        if (fault_on && (scenario->kind == SIM_FAULT_STALL)) {
            sim_plant_freeze(&plant);
        } else if (fault_on && (scenario->kind == SIM_FAULT_OVERSPEED)) {
            sim_plant_force_omega(&plant, 25.f, AEGIS_DT_S);
        } else {
            sim_plant_step(&plant, applied, AEGIS_DT_S);
        }

        memset(&in, 0, sizeof(in));
        in.now_us = (uint64_t)tick * (uint64_t)AEGIS_PERIOD_US;
        in.enc_count = plant.enc_count;
        in.gyro_rad_s = plant.omega_rad_s;
        if (fault_on && (scenario->kind == SIM_FAULT_GYRO_BIAS)) {
            in.gyro_rad_s += 5.0f;
        }
        in.hb_valid = true;
        if (fault_on && (scenario->kind == SIM_FAULT_HB_DROP)) {
            in.hb_valid = false;
        }
        if (tick == AEGIS_ARM_TICK) {
            in.cmd = AEGIS_CMD_ARM;
        } else if (tick > AEGIS_ARM_TICK) {
            reference_at(scenario, tick, &plant, fault_on, &ref, &ref_vel);
            in.cmd = AEGIS_CMD_TRAJ;
            in.ref_pos_rad = ref;
            in.ref_vel_rad_s = ref_vel;
        }

        if ((inputs != NULL) && (tick < inputs_cap)) {
            inputs[tick] = in;
        }
        aegis_step(&ctx, &in, &out);
        if ((trace != NULL) && (tick < trace_cap)) {
            trace[tick] = out.telem;
        }

        if ((out.act.duty_permille != 0) && (out.telem.state != (uint8_t)AEGIS_STATE_RUNNING)) {
            summary->duty_outside_running = true;
        }
        if ((out.telem.fault != (uint8_t)AEGIS_FAULT_NONE) && (out.act.duty_permille != 0)) {
            summary->duty_outside_running = true;
        }
        if (out.act.duty_permille != 0) {
            summary->nonzero_duty_ticks += 1u;
        }
        if (out.telem.state == (uint8_t)AEGIS_STATE_RUNNING) {
            abs_err = out.telem.ref_rad - out.telem.theta_rad;
            if (abs_err < 0.f) {
                abs_err = -abs_err;
            }
            if (abs_err > summary->max_abs_track_err_rad) {
                summary->max_abs_track_err_rad = abs_err;
            }
        }
        if (!saw_trip && (out.telem.state == (uint8_t)AEGIS_STATE_TRIPPED)) {
            saw_trip = true;
            summary->tripped = true;
            summary->trip_tick = tick;
            summary->fault = out.telem.fault;
        }
        if (saw_trip && (tick > summary->trip_tick) && (out.act.duty_permille != 0)) {
            summary->motion_after_trip = true;
        }

        if (aegis_spsc_push(&ring, &out.telem)) {
            summary->spsc_pushed += 1u;
        }
        applied = (float)out.act.duty_permille / 1000.f;
        summary->n_ticks = tick + 1u;
        summary->final_theta_rad = out.telem.theta_rad;
        summary->final_ref_rad = out.telem.ref_rad;
    }
    summary->spsc_dropped = ring.dropped;
}

void sim_run(const sim_scenario_t *scenario, sim_summary_t *summary, aegis_telem_t *trace,
             uint32_t trace_cap)
{
    run_scenario(scenario, summary, trace, trace_cap, NULL, 0u);
}

uint32_t sim_record_inputs(const sim_scenario_t *scenario, aegis_inputs_t *inputs, uint32_t cap)
{
    sim_summary_t sum;

    if ((scenario == NULL) || (inputs == NULL)) {
        return 0u;
    }
    run_scenario(scenario, &sum, NULL, 0u, inputs, cap);
    return (sum.n_ticks < cap) ? sum.n_ticks : cap;
}

static void threshold_text(sim_fault_kind_t kind, char *buf, size_t cap)
{
    switch (kind) {
    case SIM_FAULT_NONE:
        snprintf(buf, cap, "n/a");
        break;
    case SIM_FAULT_HB_DROP:
        snprintf(buf, cap, "heartbeat older than %u us", AEGIS_HB_TIMEOUT_US);
        break;
    case SIM_FAULT_STALL:
        snprintf(buf, cap, "|duty| >= %.2f and |sum encoder deltas| <= %d over %u ticks",
                 (double)AEGIS_STALL_DUTY, AEGIS_STALL_COUNT_SUM, AEGIS_STALL_TICKS);
        break;
    case SIM_FAULT_GYRO_BIAS:
        snprintf(buf, cap, "|w_enc-w_gyro| > max(%.2f, %.2f*max|w|) for %u ticks",
                 (double)AEGIS_OMEGA_DB_RAD_S, (double)AEGIS_DISAGREE_FRAC, AEGIS_DISAGREE_TICKS);
        break;
    case SIM_FAULT_OVERSPEED:
        snprintf(buf, cap, "|w_hat| > %.1f rad/s for %u tick", (double)AEGIS_V_MAX_RAD_S,
                 AEGIS_V_TICKS);
        break;
    case SIM_FAULT_TRACKING:
        snprintf(buf, cap, "|ref-theta| > %.2f rad for %u ticks", (double)AEGIS_TRACK_EPS_RAD,
                 AEGIS_TRACK_TICKS);
        break;
    default:
        snprintf(buf, cap, "unknown");
        break;
    }
}

static const char *detected_by(sim_fault_kind_t kind)
{
    switch (kind) {
    case SIM_FAULT_NONE:
        return "none";
    case SIM_FAULT_HB_DROP:
        return "heartbeat";
    case SIM_FAULT_STALL:
        return "stall";
    case SIM_FAULT_GYRO_BIAS:
        return "encoder/IMU disagreement";
    case SIM_FAULT_OVERSPEED:
        return "velocity";
    case SIM_FAULT_TRACKING:
        return "tracking error";
    default:
        return "unknown";
    }
}

int sim_write_report(const char *path, const char *cmdline)
{
    FILE *fp;
    uint32_t i;
    int bad = 0;

    if ((path == NULL) || (cmdline == NULL)) {
        return 1;
    }
    (void)mkdir("results", (mode_t)0755);
    (void)mkdir("results/sim", (mode_t)0755);
    fp = fopen(path, "w");
    if (fp == NULL) {
        return 1;
    }

    fprintf(fp, "# provenance=sim-virtual\n");
    fprintf(fp, "# cmd=%s\n", cmdline);
    fprintf(fp, "# period_us=%u\n", AEGIS_PERIOD_US);
    fprintf(fp, "# latency_us=(trip_tick-onset_tick)*period_us\n");
    fprintf(fp,
            "scenario,detected_by,threshold,onset_tick,trip_tick,latency_us,fault_code,tripped,"
            "motion_after_trip,duty_outside_running,final_theta_rad,max_abs_track_err_rad,"
            "nonzero_duty_ticks,spsc_pushed,spsc_dropped\n");

    for (i = 0u; i < sim_scenario_count(); i++) {
        const sim_scenario_t *sc = sim_scenario_at(i);
        sim_summary_t sum;
        char thresh[160];

        sim_run(sc, &sum, NULL, 0u);
        threshold_text(sc->kind, thresh, sizeof(thresh));
        if (sum.duty_outside_running || sum.motion_after_trip) {
            bad = 1;
        }
        if (sum.tripped) {
            uint32_t latency = (sum.trip_tick - sum.onset_tick) * AEGIS_PERIOD_US;
            fprintf(fp, "%s,%s,\"%s\",%u,%u,%u,%s,1,%d,%d,%.6f,%.6f,%u,%u,%u\n", sc->name,
                    detected_by(sc->kind), thresh, sum.onset_tick, sum.trip_tick, latency,
                    aegis_fault_name((aegis_fault_t)sum.fault), sum.motion_after_trip ? 1 : 0,
                    sum.duty_outside_running ? 1 : 0, (double)sum.final_theta_rad,
                    (double)sum.max_abs_track_err_rad, sum.nonzero_duty_ticks, sum.spsc_pushed,
                    sum.spsc_dropped);
        } else {
            fprintf(fp, "%s,%s,\"%s\",,,,NONE,0,%d,%d,%.6f,%.6f,%u,%u,%u\n", sc->name,
                    detected_by(sc->kind), thresh, sum.motion_after_trip ? 1 : 0,
                    sum.duty_outside_running ? 1 : 0, (double)sum.final_theta_rad,
                    (double)sum.max_abs_track_err_rad, sum.nonzero_duty_ticks, sum.spsc_pushed,
                    sum.spsc_dropped);
        }
    }

    if (fclose(fp) != 0) {
        return 1;
    }
    return bad;
}

int sim_write_traces(const char *dir)
{
    static aegis_telem_t trace[SIM_TRACE_MAX];
    uint32_t i;
    uint32_t t;

    if (dir == NULL) {
        return 1;
    }
    (void)mkdir(dir, (mode_t)0755);

    for (i = 0u; i < sim_scenario_count(); i++) {
        const sim_scenario_t *sc = sim_scenario_at(i);
        sim_summary_t sum;
        char path[512];
        FILE *fp;
        int n;

        sim_run(sc, &sum, trace, SIM_TRACE_MAX);
        n = snprintf(path, sizeof(path), "%s/%s.csv", dir, sc->name);
        if ((n < 0) || ((size_t)n >= sizeof(path))) {
            return 1;
        }
        fp = fopen(path, "w");
        if (fp == NULL) {
            return 1;
        }
        fprintf(fp, "# provenance=sim-virtual\n");
        fprintf(fp, "# scenario=%s onset_tick=%u period_us=%u\n", sc->name, sc->onset_tick,
                AEGIS_PERIOD_US);
        fprintf(fp, "tick,fault_injected,theta_rad,omega_rad_s,ref_rad,duty_permille,state,fault\n");
        for (t = 0u; (t < sum.n_ticks) && (t < SIM_TRACE_MAX); t++) {
            int injected = (sc->kind != SIM_FAULT_NONE) && (t >= sc->onset_tick);
            fprintf(fp, "%u,%d,%.6f,%.6f,%.6f,%d,%u,%s\n", trace[t].tick, injected,
                    (double)trace[t].theta_rad, (double)trace[t].omega_rad_s,
                    (double)trace[t].ref_rad, (int)trace[t].duty_permille,
                    (unsigned)trace[t].state, aegis_fault_name((aegis_fault_t)trace[t].fault));
        }
        if (fclose(fp) != 0) {
            return 1;
        }
    }
    return 0;
}
