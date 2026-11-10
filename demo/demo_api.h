#ifndef DEMO_API_H
#define DEMO_API_H

#define DEMO_SIDE_NOGATE 0
#define DEMO_SIDE_GATED 1

void demo_arm_reset(void);
void demo_arm_break(int fault);
void demo_arm_tick(void);

int demo_arm_tick_count(void);
int demo_arm_fault(void);
int demo_arm_fault_tick(void);
float demo_arm_angle(int side, int joint);
float demo_arm_speed(int side, int joint);
int demo_arm_duty(int side, int joint);
float demo_arm_link(int joint);

/* the rest are about the gated arm only */
int demo_arm_state(int joint);
int demo_arm_tripped(void);
int demo_arm_trip_fault(void);
int demo_arm_trip_tick(void);

#endif
