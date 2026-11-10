#include "sim/lockstep.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    const char *out_path = "results/sim/fault_trips.csv";
    const char *trace_dir = NULL;
    char cmdline[512];
    int i;
    int n;

    for (i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--out") == 0) && ((i + 1) < argc)) {
            i += 1;
            out_path = argv[i];
        } else if ((strcmp(argv[i], "--trace") == 0) && ((i + 1) < argc)) {
            i += 1;
            trace_dir = argv[i];
        } else {
            fprintf(stderr, "usage: aegis_lockstep [--out path] [--trace dir]\n");
            return 2;
        }
    }

    n = snprintf(cmdline, sizeof(cmdline), "%s --out %s", argv[0], out_path);
    if ((n < 0) || ((size_t)n >= sizeof(cmdline))) {
        fprintf(stderr, "command line too long\n");
        return 2;
    }
    if (sim_write_report(out_path, cmdline) != 0) {
        fprintf(stderr, "failed to write %s\n", out_path);
        return 1;
    }
    printf("wrote %s\n", out_path);
    if (trace_dir != NULL) {
        if (sim_write_traces(trace_dir) != 0) {
            fprintf(stderr, "failed to write traces to %s\n", trace_dir);
            return 1;
        }
        printf("wrote traces to %s\n", trace_dir);
    }
    return 0;
}
