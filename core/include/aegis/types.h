#ifndef AEGIS_TYPES_H
#define AEGIS_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    AEGIS_OK = 0,
    AEGIS_E_ARG = 1,
    AEGIS_E_RANGE = 2,
    AEGIS_E_FULL = 3,
    AEGIS_E_EMPTY = 4,
    AEGIS_E_STATE = 5
} aegis_status_t;

typedef enum {
    AEGIS_STATE_INIT = 0,
    AEGIS_STATE_ARMED = 1,
    AEGIS_STATE_RUNNING = 2,
    AEGIS_STATE_TRIPPED = 3
} aegis_state_t;

typedef enum {
    AEGIS_FAULT_NONE = 0,
    AEGIS_FAULT_TRACKING = 1,
    AEGIS_FAULT_VELOCITY = 2,
    AEGIS_FAULT_HEARTBEAT = 3,
    AEGIS_FAULT_DISAGREE = 4,
    AEGIS_FAULT_STALL = 5,
    AEGIS_FAULT_NUMERIC = 6,
    AEGIS_FAULT_INTERNAL = 7
} aegis_fault_t;

typedef enum {
    AEGIS_DRIVE_BRAKE = 0,
    AEGIS_DRIVE_FWD = 1,
    AEGIS_DRIVE_REV = 2
} aegis_drive_mode_t;

typedef enum {
    AEGIS_CMD_NONE = 0,
    AEGIS_CMD_HEARTBEAT = 1,
    AEGIS_CMD_ARM = 2,
    AEGIS_CMD_STOP = 3,
    AEGIS_CMD_RESET = 4,
    AEGIS_CMD_TRAJ = 5
} aegis_cmd_type_t;

#define AEGIS_BIT_TRACKING (1u << 0)
#define AEGIS_BIT_VELOCITY (1u << 1)
#define AEGIS_BIT_HEARTBEAT (1u << 2)
#define AEGIS_BIT_DISAGREE (1u << 3)
#define AEGIS_BIT_STALL (1u << 4)
#define AEGIS_BIT_NUMERIC (1u << 5)
#define AEGIS_BIT_INTERNAL (1u << 6)

typedef struct {
    aegis_drive_mode_t mode;
    int16_t duty_permille;
} aegis_actuation_t;

typedef struct {
    uint32_t tick;
    float theta_rad;
    float omega_rad_s;
    float ref_rad;
    int16_t duty_permille;
    uint8_t state;
    uint8_t fault;
} aegis_telem_t;

static inline bool aegis_is_finite(float x)
{
    return (x <= 1.0e30f) && (x >= -1.0e30f);
}

const char *aegis_fault_name(aegis_fault_t fault);
aegis_fault_t aegis_fault_from_mask(uint32_t mask);

#endif
