#ifndef SIM_LOCKSTEP_H
#define SIM_LOCKSTEP_H

#include "aegis/safety.h"

#include <stdbool.h>
#include <stdint.h>

#define SIM_TRACE_MAX 512u

typedef enum {
    SIM_FAULT_NONE = 0,
    SIM_FAULT_HB_DROP = 1,
    SIM_FAULT_STALL = 2,
    SIM_FAULT_GYRO_BIAS = 3,
    SIM_FAULT_OVERSPEED = 4,
    SIM_FAULT_TRACKING = 5
} sim_fault_kind_t;

typedef struct {
    const char *name;
    sim_fault_kind_t kind;
    uint32_t onset_tick;
    uint32_t duration_ticks;
    aegis_fault_t expect_fault;
} sim_scenario_t;

typedef struct {
    bool tripped;
    bool motion_after_trip;
    bool duty_outside_running;
    uint8_t fault;
    uint32_t trip_tick;
    uint32_t onset_tick;
    uint32_t n_ticks;
    uint32_t spsc_pushed;
    uint32_t spsc_dropped;
    uint32_t nonzero_duty_ticks;
    float final_theta_rad;
    float final_ref_rad;
    float max_abs_track_err_rad;
} sim_summary_t;

uint32_t sim_scenario_count(void);
const sim_scenario_t *sim_scenario_at(uint32_t index);
void sim_run(const sim_scenario_t *scenario, sim_summary_t *summary, aegis_telem_t *trace,
             uint32_t trace_cap);
uint32_t sim_record_inputs(const sim_scenario_t *scenario, aegis_inputs_t *inputs, uint32_t cap);
int sim_write_report(const char *path, const char *cmdline);
int sim_write_traces(const char *dir);

#endif
