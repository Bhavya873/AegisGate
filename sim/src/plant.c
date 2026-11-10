#include "sim/plant.h"

#include "aegis_config.h"

#include <stddef.h>

static int32_t enc_from_theta(float theta_rad)
{
    float scaled = theta_rad / AEGIS_K_C_RAD;
    int32_t trunc;

    if (!(scaled < 0.f)) {
        if (scaled > 2000000000.f) {
            return 2000000000;
        }
        return (int32_t)scaled;
    }
    if (scaled < -2000000000.f) {
        return -2000000000;
    }
    trunc = (int32_t)scaled;
    if ((float)trunc > scaled) {
        return trunc - 1;
    }
    return trunc;
}

void sim_plant_init(sim_plant_t *plant)
{
    if (plant == NULL) {
        return;
    }
    plant->theta_rad = 0.f;
    plant->omega_rad_s = 0.f;
    plant->enc_count = 0;
}

void sim_plant_step(sim_plant_t *plant, float duty, float dt_s)
{
    float tau;

    if (plant == NULL) {
        return;
    }
    tau = duty * SIM_PLANT_KT - SIM_PLANT_B * plant->omega_rad_s;
    plant->omega_rad_s += dt_s * tau / SIM_PLANT_J;
    plant->theta_rad += dt_s * plant->omega_rad_s;
    plant->enc_count = enc_from_theta(plant->theta_rad);
}

void sim_plant_freeze(sim_plant_t *plant)
{
    if (plant == NULL) {
        return;
    }
    plant->omega_rad_s = 0.f;
    plant->enc_count = enc_from_theta(plant->theta_rad);
}

void sim_plant_force_omega(sim_plant_t *plant, float omega_rad_s, float dt_s)
{
    if (plant == NULL) {
        return;
    }
    plant->omega_rad_s = omega_rad_s;
    plant->theta_rad += dt_s * omega_rad_s;
    plant->enc_count = enc_from_theta(plant->theta_rad);
}
