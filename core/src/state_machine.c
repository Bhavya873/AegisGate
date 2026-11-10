#include "aegis/state_machine.h"

void aegis_sm_init(aegis_sm_t *sm)
{
    if (sm == NULL) {
        return;
    }
    aegis_sm_set(sm, AEGIS_STATE_INIT);
}

bool aegis_sm_verify(const aegis_sm_t *sm)
{
    uint32_t expect;

    if (sm == NULL) {
        return false;
    }
    expect = ~((uint32_t)sm->state);
    return sm->state_inv == expect;
}

void aegis_sm_set(aegis_sm_t *sm, aegis_state_t state)
{
    if (sm == NULL) {
        return;
    }
    sm->state = state;
    sm->state_inv = ~((uint32_t)state);
}

void aegis_sm_apply(aegis_sm_t *sm, aegis_event_t ev)
{
    if (sm == NULL) {
        return;
    }
    if (!aegis_sm_verify(sm)) {
        aegis_sm_set(sm, AEGIS_STATE_TRIPPED);
        return;
    }

    switch (sm->state) {
    case AEGIS_STATE_INIT:
        if (ev == AEGIS_EV_SELFTEST_OK) {
            aegis_sm_set(sm, AEGIS_STATE_ARMED);
        } else if (ev == AEGIS_EV_FAULT) {
            aegis_sm_set(sm, AEGIS_STATE_TRIPPED);
        }
        break;
    case AEGIS_STATE_ARMED:
        if (ev == AEGIS_EV_ARM) {
            aegis_sm_set(sm, AEGIS_STATE_RUNNING);
        } else if (ev == AEGIS_EV_FAULT) {
            aegis_sm_set(sm, AEGIS_STATE_TRIPPED);
        }
        break;
    case AEGIS_STATE_RUNNING:
        if (ev == AEGIS_EV_STOP) {
            aegis_sm_set(sm, AEGIS_STATE_ARMED);
        } else if (ev == AEGIS_EV_FAULT) {
            aegis_sm_set(sm, AEGIS_STATE_TRIPPED);
        }
        break;
    case AEGIS_STATE_TRIPPED:
        if (ev == AEGIS_EV_RESET) {
            aegis_sm_set(sm, AEGIS_STATE_ARMED);
        }
        break;
    default:
        aegis_sm_set(sm, AEGIS_STATE_TRIPPED);
        break;
    }
}
