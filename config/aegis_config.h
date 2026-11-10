#ifndef AEGIS_CONFIG_H
#define AEGIS_CONFIG_H

#define AEGIS_PERIOD_US 5000u
#define AEGIS_DT_S 0.005f

#define AEGIS_TWO_PI 6.2831853f
#define AEGIS_ENC_COUNTS_PER_REV 1200
#define AEGIS_K_C_RAD (AEGIS_TWO_PI / (float)AEGIS_ENC_COUNTS_PER_REV)

#define AEGIS_TRACK_EPS_RAD 0.20f
#define AEGIS_TRACK_TICKS 2u

#define AEGIS_V_MAX_RAD_S 12.0f
#define AEGIS_V_TICKS 1u

#define AEGIS_HB_TIMEOUT_US 50000u
#define AEGIS_HB_TICKS 1u

#define AEGIS_OMEGA_DB_RAD_S 1.5f
#define AEGIS_DISAGREE_FRAC 0.25f
#define AEGIS_DISAGREE_TICKS 4u

#define AEGIS_STALL_DUTY 0.60f
#define AEGIS_STALL_TICKS 3u
#define AEGIS_STALL_COUNT_SUM 1

#define AEGIS_DUTY_MAX 0.95f
#define AEGIS_PID_KP 6.0f
#define AEGIS_PID_KI 0.5f
#define AEGIS_PID_KD 0.08f
#define AEGIS_PID_I_CLAMP 0.30f

#define AEGIS_COMP_TAU_S 0.05f

#define AEGIS_SPSC_CAP 64u

#define AEGIS_ARM_TICK 5u
#define AEGIS_RAMP_START_TICK 40u
#define AEGIS_RAMP_END_TICK 240u
#define AEGIS_RAMP_END_RAD 0.50f
#define AEGIS_RAMP_VEL_RAD_S 0.50f

#define AEGIS_ARM_JOINTS 2
#define AEGIS_ARM_LINK0_M 0.30f
#define AEGIS_ARM_LINK1_M 0.22f
#define AEGIS_ARM_POSE_A0_RAD -0.80f
#define AEGIS_ARM_POSE_A1_RAD 0.70f
#define AEGIS_ARM_POSE_B0_RAD 0.80f
#define AEGIS_ARM_POSE_B1_RAD -0.70f
#define AEGIS_ARM_SWEEP_S 2.0f
#define AEGIS_ARM_FAULT_JOINT 0
#define AEGIS_ARM_RUNAWAY_RAD_S 25.0f
#define AEGIS_ARM_FAULT_ONSET_TICK 300u
#define AEGIS_ARM_RUN_TICKS 800u

_Static_assert(AEGIS_PERIOD_US == 5000u, "lockstep period is 5 ms");
_Static_assert(AEGIS_STALL_TICKS >= 1u, "stall window");
_Static_assert(AEGIS_TRACK_TICKS >= 1u, "tracking debounce");
_Static_assert(AEGIS_V_TICKS >= 1u, "velocity debounce");
_Static_assert(AEGIS_HB_TICKS >= 1u, "heartbeat debounce");
_Static_assert(AEGIS_DISAGREE_TICKS >= 1u, "disagreement debounce");
_Static_assert(AEGIS_HB_TIMEOUT_US > AEGIS_PERIOD_US, "heartbeat longer than one tick");
_Static_assert((AEGIS_SPSC_CAP & (AEGIS_SPSC_CAP - 1u)) == 0u, "spsc capacity");
_Static_assert(AEGIS_ENC_COUNTS_PER_REV > 0, "encoder counts");
_Static_assert(AEGIS_RAMP_END_TICK > AEGIS_RAMP_START_TICK, "ramp");
_Static_assert(AEGIS_ARM_FAULT_JOINT < AEGIS_ARM_JOINTS, "arm fault joint");
_Static_assert(AEGIS_ARM_RUN_TICKS > AEGIS_ARM_FAULT_ONSET_TICK, "arm run length");

#endif
