#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdint.h>

#define SIM_PLANT_J 0.005f
#define SIM_PLANT_B 0.02f
#define SIM_PLANT_KT 0.40f

typedef struct {
    float theta_rad;
    float omega_rad_s;
    int32_t enc_count;
} sim_plant_t;

void sim_plant_init(sim_plant_t *plant);
void sim_plant_step(sim_plant_t *plant, float duty, float dt_s);
void sim_plant_freeze(sim_plant_t *plant);
void sim_plant_force_omega(sim_plant_t *plant, float omega_rad_s, float dt_s);

#endif
