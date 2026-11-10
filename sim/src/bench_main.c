#include "sim/lockstep.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <time.h>

#ifndef AEGIS_BUILD_TYPE
#define AEGIS_BUILD_TYPE "unknown"
#endif

#define BENCH_BATCHES 4000u

static aegis_inputs_t g_inputs[SIM_TRACE_MAX];
static aegis_telem_t g_ref_trace[SIM_TRACE_MAX];
static double g_batch_ns[BENCH_BATCHES];
static volatile int32_t g_sink;

static uint64_t now_ns(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000ull) + (uint64_t)ts.tv_nsec;
}

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a;
    double y = *(const double *)b;
    return (x > y) - (x < y);
}

static size_t pct_index(size_t n, size_t pct)
{
    size_t i = (n * pct) / 100u;
    return (i >= n) ? (n - 1u) : i;
}

/* Replay the recorded inputs open loop. aegis_step is deterministic, so the
 * telemetry must match the closed-loop trace exactly. */
static int verify_replay(uint32_t n)
{
    aegis_ctx_t ctx;
    uint32_t t;

    aegis_init(&ctx);
    for (t = 0u; t < n; t++) {
        aegis_outputs_t out;
        aegis_step(&ctx, &g_inputs[t], &out);
        if (memcmp(&out.telem, &g_ref_trace[t], sizeof(out.telem)) != 0) {
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *out_path = "results/bench/step_timing.csv";
    const sim_scenario_t *nominal = sim_scenario_at(0u);
    sim_summary_t sum;
    struct utsname host;
    uint32_t n;
    uint32_t b;
    uint32_t t;
    FILE *fp;
    double batch_med;
    double batch_p99;
    double batch_max;

    if ((argc == 3) && (strcmp(argv[1], "--out") == 0)) {
        out_path = argv[2];
    } else if (argc != 1) {
        fprintf(stderr, "usage: aegis_bench [--out path]\n");
        return 2;
    }

    n = sim_record_inputs(nominal, g_inputs, SIM_TRACE_MAX);
    sim_run(nominal, &sum, g_ref_trace, SIM_TRACE_MAX);
    if ((n == 0u) || (verify_replay(n) != 0)) {
        fprintf(stderr, "replay mismatch: inputs do not reproduce the nominal trace\n");
        return 1;
    }

    /* Batch timing: one full nominal run per batch, timer outside the loop.
     * Per-call timing is not reported: CLOCK_MONOTONIC on macOS ticks in 1 us. */
    for (b = 0u; b < BENCH_BATCHES; b++) {
        aegis_ctx_t ctx;
        uint64_t t0;
        uint64_t t1;

        aegis_init(&ctx);
        t0 = now_ns();
        for (t = 0u; t < n; t++) {
            aegis_outputs_t out;
            aegis_step(&ctx, &g_inputs[t], &out);
            g_sink += out.act.duty_permille;
        }
        t1 = now_ns();
        g_batch_ns[b] = (double)(t1 - t0) / (double)n;
    }

    qsort(g_batch_ns, BENCH_BATCHES, sizeof(g_batch_ns[0]), cmp_double);
    batch_med = g_batch_ns[pct_index(BENCH_BATCHES, 50u)];
    batch_p99 = g_batch_ns[pct_index(BENCH_BATCHES, 99u)];
    batch_max = g_batch_ns[BENCH_BATCHES - 1u];

    if (uname(&host) != 0) {
        memset(&host, 0, sizeof(host));
    }

    printf("aegis_step wall clock, host CPU, %s build (%s %s)\n", AEGIS_BUILD_TYPE, host.sysname,
           host.machine);
    printf("  replay check            : %u ticks reproduce the nominal trace exactly\n", n);
    printf("  batch mean per step     : median %.1f ns, p99 %.1f ns, max %.1f ns (%u batches x %u steps)\n",
           batch_med, batch_p99, batch_max, BENCH_BATCHES, n);
    printf("  budget                  : %u ns per 5 ms tick\n", AEGIS_PERIOD_US * 1000u);

    (void)mkdir("results", (mode_t)0755);
    (void)mkdir("results/bench", (mode_t)0755);
    fp = fopen(out_path, "w");
    if (fp == NULL) {
        fprintf(stderr, "failed to write %s\n", out_path);
        return 1;
    }
    fprintf(fp, "# provenance=host-wall-clock (not hardware, not a real-time OS)\n");
    fprintf(fp, "# cmd=%s%s%s\n", argv[0], (argc == 3) ? " --out " : "", (argc == 3) ? argv[2] : "");
    fprintf(fp, "# build=%s host=%s %s\n", AEGIS_BUILD_TYPE, host.sysname, host.machine);
    fprintf(fp, "# workload=nominal scenario inputs replayed open loop, %u steps\n", n);
    fprintf(fp, "metric,median_ns,p99_ns,max_ns,batches\n");
    fprintf(fp, "batch_mean_per_step,%.1f,%.1f,%.1f,%u\n", batch_med, batch_p99, batch_max,
            BENCH_BATCHES);
    if (fclose(fp) != 0) {
        return 1;
    }
    printf("wrote %s\n", out_path);
    return 0;
}
