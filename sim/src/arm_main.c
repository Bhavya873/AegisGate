#include "sim/arm.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    const char *path = "results/sim/arm_trips.csv";
    FILE *fp;
    int f;

    if (argc == 3 && strcmp(argv[1], "--out") == 0) {
        path = argv[2];
    } else if (argc != 1) {
        fprintf(stderr, "usage: aegis_arm [--out path]\n");
        return 2;
    }

    fp = fopen(path, "w");
    if (fp == NULL) {
        fprintf(stderr, "failed to open %s\n", path);
        return 1;
    }
    fprintf(fp, "# provenance=sim-virtual\n");
    fprintf(fp, "# period_us=%u ticks=%u fault_joint=%d\n", AEGIS_PERIOD_US, AEGIS_ARM_RUN_TICKS,
            AEGIS_ARM_FAULT_JOINT);
    fprintf(fp, "# latency_us=(trip_tick-onset_tick)*period_us\n");
    fprintf(fp, "fault,onset_tick,gated_tripped,gated_check,trip_tick,latency_us,"
                "gated_max_duty_after_trip,gated_final_speed_rad_s,nogate_max_duty_after_onset,"
                "nogate_final_duty,nogate_final_speed_rad_s,nogate_final_angle_rad\n");

    for (f = 0; f < SIM_ARM_FAULT_COUNT; f++) {
        sim_arm_summary_t s;

        sim_arm_run((sim_arm_fault_t)f, &s);
        fprintf(fp, "%s,%u,%d,%s,", sim_arm_fault_name((sim_arm_fault_t)f), s.onset_tick,
                s.gated_tripped ? 1 : 0, aegis_fault_name((aegis_fault_t)s.gated_fault));
        if (s.gated_tripped) {
            fprintf(fp, "%u,%u,", s.trip_tick, (s.trip_tick - s.onset_tick) * AEGIS_PERIOD_US);
        } else {
            fprintf(fp, ",,");
        }
        fprintf(fp, "%d,%.6f,%d,%d,%.6f,%.6f\n", s.gated_max_duty_after_trip,
                (double)s.gated_final_speed, s.nogate_max_duty_after_onset, s.nogate_final_duty,
                (double)s.nogate_final_speed, (double)s.nogate_final_angle);
    }

    if (fclose(fp) != 0) {
        return 1;
    }
    printf("wrote %s\n", path);
    return 0;
}
