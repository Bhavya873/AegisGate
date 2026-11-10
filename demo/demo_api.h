#ifndef DEMO_API_H
#define DEMO_API_H

#define DEMO_FAULT_HEARTBEAT 0
#define DEMO_FAULT_JAM 1
#define DEMO_FAULT_GYRO_BIAS 2
#define DEMO_FAULT_SPIN 3
#define DEMO_FAULT_GYRO_NAN 4
#define DEMO_FAULT_COUNT 5

#define DEMO_CMD_ARM 1
#define DEMO_CMD_STOP 2
#define DEMO_CMD_RESET 3

void demo_reset(void);
void demo_send(int cmd);
void demo_set_target(float target_rad);
void demo_set_fault(int fault, int on);
void demo_jump_ref(void);
void demo_tick(void);

/* demo_pop() takes the next telemetry sample off the ring, the getters read that sample */
int demo_pop(void);
int demo_tick_count(void);
float demo_angle(void);
float demo_ref(void);
float demo_speed(void);
int demo_duty(void);
int demo_state(void);
int demo_fault(void);
int demo_dropped(void);

#endif
