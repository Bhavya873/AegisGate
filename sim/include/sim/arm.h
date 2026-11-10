#ifndef SIM_ARM_H
#define SIM_ARM_H

#include "aegis/safety.h"
#include "aegis_config.h"
#include "sim/plant.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SIM_ARM_FAULT_NONE = 0,
    SIM_ARM_FAULT_JAM = 1,
    SIM_ARM_FAULT_SENSOR = 2,
    SIM_ARM_FAULT_SILENT = 3,
    SIM_ARM_FAULT_RUNAWAY = 4
} sim_arm_fault_t;

#define SIM_ARM_FAULT_COUNT 5

/* what the host sends this tick, plus the fault being injected */
typedef struct {
    float ref[AEGIS_ARM_JOINTS];
    float ref_vel[AEGIS_ARM_JOINTS];
    bool alive;
    sim_arm_fault_t fault;
} sim_arm_cmd_t;

typedef struct {
    float phase;
    float dir;
    float ref[AEGIS_ARM_JOINTS];
} sim_arm_host_t;

typedef struct {
    aegis_ctx_t gate[AEGIS_ARM_JOINTS];
    sim_plant_t joint[AEGIS_ARM_JOINTS];
    float applied[AEGIS_ARM_JOINTS];
    aegis_telem_t telem[AEGIS_ARM_JOINTS];
    int32_t frozen_count;
    bool frozen;
    bool tripped;
    uint32_t trip_tick;
    uint8_t trip_fault;
} sim_arm_gated_t;

/* same estimator and PID as the gated arm but nothing checks the output */
typedef struct {
    aegis_est_t est[AEGIS_ARM_JOINTS];
    aegis_pid_t pid[AEGIS_ARM_JOINTS];
    sim_plant_t joint[AEGIS_ARM_JOINTS];
    int16_t duty_permille[AEGIS_ARM_JOINTS];
    int32_t frozen_count;
    bool frozen;
} sim_arm_nogate_t;

typedef struct {
    bool gated_tripped;
    uint8_t gated_fault;
    uint32_t onset_tick;
    uint32_t trip_tick;
    int gated_max_duty_after_trip;
    float gated_final_speed;
    int nogate_max_duty_after_onset;
    int nogate_final_duty;
    float nogate_final_speed;
    float nogate_final_angle;
} sim_arm_summary_t;

const char *sim_arm_fault_name(sim_arm_fault_t fault);

void sim_arm_host_init(sim_arm_host_t *host);
void sim_arm_host_step(sim_arm_host_t *host, uint32_t tick, sim_arm_fault_t fault,
                       sim_arm_cmd_t *cmd);

void sim_arm_gated_init(sim_arm_gated_t *arm);
void sim_arm_gated_step(sim_arm_gated_t *arm, uint32_t tick, const sim_arm_cmd_t *cmd);

void sim_arm_nogate_init(sim_arm_nogate_t *arm);
void sim_arm_nogate_step(sim_arm_nogate_t *arm, uint32_t tick, const sim_arm_cmd_t *cmd);

void sim_arm_run(sim_arm_fault_t fault, sim_arm_summary_t *summary);

#endif
