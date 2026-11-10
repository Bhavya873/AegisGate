#ifndef AEGIS_PID_H
#define AEGIS_PID_H

#include "aegis/types.h"

typedef struct {
    float integ;
} aegis_pid_t;

void aegis_pid_init(aegis_pid_t *pid);
float aegis_pid_step(aegis_pid_t *pid, float error_rad, float omega_hat, float omega_ref);

#endif
