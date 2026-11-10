#ifndef AEGIS_STATE_MACHINE_H
#define AEGIS_STATE_MACHINE_H

#include "aegis/types.h"

typedef enum {
    AEGIS_EV_NONE = 0,
    AEGIS_EV_SELFTEST_OK = 1,
    AEGIS_EV_ARM = 2,
    AEGIS_EV_STOP = 3,
    AEGIS_EV_FAULT = 4,
    AEGIS_EV_RESET = 5
} aegis_event_t;

typedef struct {
    aegis_state_t state;
    uint32_t state_inv;
} aegis_sm_t;

void aegis_sm_init(aegis_sm_t *sm);
bool aegis_sm_verify(const aegis_sm_t *sm);
void aegis_sm_set(aegis_sm_t *sm, aegis_state_t state);
void aegis_sm_apply(aegis_sm_t *sm, aegis_event_t ev);

#endif
