#ifndef AEGIS_ARBITER_H
#define AEGIS_ARBITER_H

#include "aegis/types.h"

aegis_actuation_t aegis_arbiter_decide(aegis_state_t state, uint32_t fault_mask, float pid_duty);

#endif
