#include "sim/arm.h"

#include <stddef.h>
#include <string.h>

static const float pose_a[AEGIS_ARM_JOINTS] = {AEGIS_ARM_POSE_A0_RAD, AEGIS_ARM_POSE_A1_RAD};
static const float pose_b[AEGIS_ARM_JOINTS] = {AEGIS_ARM_POSE_B0_RAD, AEGIS_ARM_POSE_B1_RAD};

const char *sim_arm_fault_name(sim_arm_fault_t fault)
{
    switch (fault) {
    case SIM_ARM_FAULT_NONE:
        return "none";
    case SIM_ARM_FAULT_JAM:
        return "joint_jam";
    case SIM_ARM_FAULT_SENSOR:
        return "encoder_frozen";
    case SIM_ARM_FAULT_SILENT:
        return "host_silent";
    case SIM_ARM_FAULT_RUNAWAY:
        return "runaway_command";
    default:
        return "unknown";
    }
}

static int abs_int(int x)
{
    if (x < 0) {
        return -x;
    }
    return x;
}

void sim_arm_host_init(sim_arm_host_t *host)
{
    int j;

    if (host == NULL) {
        return;
    }
    /* phase 0.5 is the middle of the sweep, which is where the joints start */
    host->phase = 0.5f;
    host->dir = 1.0f;
    for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
        host->ref[j] = pose_a[j] + 0.5f * (pose_b[j] - pose_a[j]);
    }
}

void sim_arm_host_step(sim_arm_host_t *host, uint32_t tick, sim_arm_fault_t fault,
                       sim_arm_cmd_t *cmd)
{
    int j;

    if (host == NULL || cmd == NULL) {
        return;
    }
    memset(cmd, 0, sizeof(*cmd));
    cmd->fault = fault;
    cmd->alive = true;

    if (fault == SIM_ARM_FAULT_SILENT) {
        cmd->alive = false;
        for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
            cmd->ref[j] = host->ref[j];
        }
        return;
    }

    if (tick > AEGIS_ARM_TICK) {
        host->phase = host->phase + host->dir * AEGIS_DT_S / AEGIS_ARM_SWEEP_S;
        if (host->phase >= 1.0f) {
            host->phase = 1.0f;
            host->dir = -1.0f;
        } else if (host->phase <= 0.0f) {
            host->phase = 0.0f;
            host->dir = 1.0f;
        }
    }

    for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
        if (fault == SIM_ARM_FAULT_RUNAWAY && j == AEGIS_ARM_FAULT_JOINT) {
            host->ref[j] = host->ref[j] + AEGIS_ARM_RUNAWAY_RAD_S * AEGIS_DT_S;
            cmd->ref_vel[j] = AEGIS_ARM_RUNAWAY_RAD_S;
        } else {
            host->ref[j] = pose_a[j] + host->phase * (pose_b[j] - pose_a[j]);
            if (tick > AEGIS_ARM_TICK) {
                cmd->ref_vel[j] = host->dir * (pose_b[j] - pose_a[j]) / AEGIS_ARM_SWEEP_S;
            }
        }
        cmd->ref[j] = host->ref[j];
    }
}

/* both arms go through these two helpers so they get exactly the same fault */
static void step_joint(sim_plant_t *joint, int j, float duty, sim_arm_fault_t fault)
{
    if (fault == SIM_ARM_FAULT_JAM && j == AEGIS_ARM_FAULT_JOINT) {
        sim_plant_freeze(joint);
    } else {
        sim_plant_step(joint, duty, AEGIS_DT_S);
    }
}

static int32_t read_encoder(const sim_plant_t *joint, int j, sim_arm_fault_t fault,
                            int32_t *frozen_count, bool *frozen)
{
    if (fault != SIM_ARM_FAULT_SENSOR || j != AEGIS_ARM_FAULT_JOINT) {
        return joint->enc_count;
    }
    if (!*frozen) {
        *frozen_count = joint->enc_count;
        *frozen = true;
    }
    return *frozen_count;
}

void sim_arm_gated_init(sim_arm_gated_t *arm)
{
    int j;

    if (arm == NULL) {
        return;
    }
    memset(arm, 0, sizeof(*arm));
    for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
        aegis_init(&arm->gate[j]);
        sim_plant_init(&arm->joint[j]);
    }
}

void sim_arm_gated_step(sim_arm_gated_t *arm, uint32_t tick, const sim_arm_cmd_t *cmd)
{
    int j;
    int k;

    if (arm == NULL || cmd == NULL) {
        return;
    }
    for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
        aegis_inputs_t in;
        aegis_outputs_t out;
        bool other_tripped = false;

        step_joint(&arm->joint[j], j, arm->applied[j], cmd->fault);

        for (k = 0; k < AEGIS_ARM_JOINTS; k++) {
            if (k != j && arm->telem[k].state == AEGIS_STATE_TRIPPED) {
                other_tripped = true;
            }
        }

        memset(&in, 0, sizeof(in));
        in.now_us = (uint64_t)tick * AEGIS_PERIOD_US;
        in.enc_count = read_encoder(&arm->joint[j], j, cmd->fault, &arm->frozen_count, &arm->frozen);
        in.gyro_rad_s = arm->joint[j].omega_rad_s;
        in.hb_valid = cmd->alive;
        if (!cmd->alive) {
            in.cmd = AEGIS_CMD_NONE;
        } else if (other_tripped) {
            /* one joint tripped, so the rest of the arm is stopped too */
            in.cmd = AEGIS_CMD_STOP;
        } else if (tick == AEGIS_ARM_TICK) {
            in.cmd = AEGIS_CMD_ARM;
        } else if (tick > AEGIS_ARM_TICK) {
            in.cmd = AEGIS_CMD_TRAJ;
            in.ref_pos_rad = cmd->ref[j];
            in.ref_vel_rad_s = cmd->ref_vel[j];
        }

        aegis_step(&arm->gate[j], &in, &out);
        arm->applied[j] = (float)out.act.duty_permille / 1000.0f;
        arm->telem[j] = out.telem;

        if (!arm->tripped && out.telem.state == AEGIS_STATE_TRIPPED) {
            arm->tripped = true;
            arm->trip_tick = tick;
            arm->trip_fault = out.telem.fault;
        }
    }
}

void sim_arm_nogate_init(sim_arm_nogate_t *arm)
{
    int j;

    if (arm == NULL) {
        return;
    }
    memset(arm, 0, sizeof(*arm));
    for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
        aegis_est_init(&arm->est[j]);
        aegis_pid_init(&arm->pid[j]);
        sim_plant_init(&arm->joint[j]);
    }
}

void sim_arm_nogate_step(sim_arm_nogate_t *arm, uint32_t tick, const sim_arm_cmd_t *cmd)
{
    int j;

    if (arm == NULL || cmd == NULL) {
        return;
    }
    for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
        int32_t enc;
        float err;
        float u;
        int duty;

        step_joint(&arm->joint[j], j, (float)arm->duty_permille[j] / 1000.0f, cmd->fault);
        enc = read_encoder(&arm->joint[j], j, cmd->fault, &arm->frozen_count, &arm->frozen);
        aegis_est_update(&arm->est[j], enc, arm->joint[j].omega_rad_s);

        /* a silent host sends nothing, so the driver keeps the last duty it was given */
        if (!cmd->alive || tick <= AEGIS_ARM_TICK) {
            continue;
        }
        err = cmd->ref[j] - arm->est[j].theta_rad;
        u = aegis_pid_step(&arm->pid[j], err, arm->est[j].omega_hat, cmd->ref_vel[j]);
        duty = (int)(u * 1000.0f);
        if (duty > 1000) {
            duty = 1000;
        }
        if (duty < -1000) {
            duty = -1000;
        }
        arm->duty_permille[j] = (int16_t)duty;
    }
}

void sim_arm_run(sim_arm_fault_t fault, sim_arm_summary_t *summary)
{
    sim_arm_host_t host;
    sim_arm_gated_t gated;
    sim_arm_nogate_t nogate;
    sim_arm_cmd_t cmd;
    uint32_t tick;
    int j;

    if (summary == NULL) {
        return;
    }
    memset(summary, 0, sizeof(*summary));
    summary->onset_tick = AEGIS_ARM_FAULT_ONSET_TICK;

    sim_arm_host_init(&host);
    sim_arm_gated_init(&gated);
    sim_arm_nogate_init(&nogate);

    for (tick = 0; tick < AEGIS_ARM_RUN_TICKS; tick++) {
        sim_arm_fault_t now = SIM_ARM_FAULT_NONE;

        if (tick >= AEGIS_ARM_FAULT_ONSET_TICK) {
            now = fault;
        }
        sim_arm_host_step(&host, tick, now, &cmd);
        sim_arm_gated_step(&gated, tick, &cmd);
        sim_arm_nogate_step(&nogate, tick, &cmd);

        for (j = 0; j < AEGIS_ARM_JOINTS; j++) {
            int g = abs_int(gated.telem[j].duty_permille);
            int n = abs_int(nogate.duty_permille[j]);

            /* the other joint hears about a trip one tick later */
            if (gated.tripped && tick > gated.trip_tick + 1 && g > summary->gated_max_duty_after_trip) {
                summary->gated_max_duty_after_trip = g;
            }
            if (tick >= AEGIS_ARM_FAULT_ONSET_TICK && n > summary->nogate_max_duty_after_onset) {
                summary->nogate_max_duty_after_onset = n;
            }
        }
    }

    summary->gated_tripped = gated.tripped;
    summary->gated_fault = gated.trip_fault;
    summary->trip_tick = gated.trip_tick;
    summary->gated_final_speed = gated.joint[AEGIS_ARM_FAULT_JOINT].omega_rad_s;
    summary->nogate_final_duty = nogate.duty_permille[AEGIS_ARM_FAULT_JOINT];
    summary->nogate_final_speed = nogate.joint[AEGIS_ARM_FAULT_JOINT].omega_rad_s;
    summary->nogate_final_angle = nogate.joint[AEGIS_ARM_FAULT_JOINT].theta_rad;
}
