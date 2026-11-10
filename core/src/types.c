#include "aegis/types.h"

_Static_assert(sizeof(aegis_telem_t) == 20u, "telemetry struct grew padding");

const char *aegis_fault_name(aegis_fault_t fault)
{
    switch (fault) {
    case AEGIS_FAULT_NONE:
        return "NONE";
    case AEGIS_FAULT_TRACKING:
        return "TRACKING";
    case AEGIS_FAULT_VELOCITY:
        return "VELOCITY";
    case AEGIS_FAULT_HEARTBEAT:
        return "HEARTBEAT";
    case AEGIS_FAULT_DISAGREE:
        return "DISAGREE";
    case AEGIS_FAULT_STALL:
        return "STALL";
    case AEGIS_FAULT_NUMERIC:
        return "NUMERIC";
    case AEGIS_FAULT_INTERNAL:
        return "INTERNAL";
    default:
        return "UNKNOWN";
    }
}

aegis_fault_t aegis_fault_from_mask(uint32_t mask)
{
    if ((mask & AEGIS_BIT_TRACKING) != 0u) {
        return AEGIS_FAULT_TRACKING;
    }
    if ((mask & AEGIS_BIT_VELOCITY) != 0u) {
        return AEGIS_FAULT_VELOCITY;
    }
    if ((mask & AEGIS_BIT_HEARTBEAT) != 0u) {
        return AEGIS_FAULT_HEARTBEAT;
    }
    if ((mask & AEGIS_BIT_DISAGREE) != 0u) {
        return AEGIS_FAULT_DISAGREE;
    }
    if ((mask & AEGIS_BIT_STALL) != 0u) {
        return AEGIS_FAULT_STALL;
    }
    if ((mask & AEGIS_BIT_NUMERIC) != 0u) {
        return AEGIS_FAULT_NUMERIC;
    }
    if ((mask & AEGIS_BIT_INTERNAL) != 0u) {
        return AEGIS_FAULT_INTERNAL;
    }
    return AEGIS_FAULT_NONE;
}
