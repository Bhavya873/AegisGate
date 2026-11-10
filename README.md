# AegisGate

Fail-closed actuator safety loop in C11. A 200 Hz virtual-time simulator checks five invariants, then one arbiter is the only code that may command a non-zero motor duty.

AegisGate is a portfolio project, not a certified safety system.

## Loop

```
 host commands                         1-DOF plant + fault injector
 heartbeat, traj, arm, stop, reset              |
        |                                       | encoder count, gyro
        v                                       v
   aegis_step  (virtual time, 5 ms ticks)
        |  complementary filter
        |  tracking, velocity, heartbeat,
        |  encoder/IMU disagreement, stall
        |  INIT / ARMED / RUNNING / TRIPPED
        v
   aegis_arbiter_decide ---- duty ------------> plant
        |
        +--> telemetry --> SPSC ring (drop if full)
```

Duty is in thousandths of full scale, in [-1000, 1000]. Zero is brake. A zeroed actuation struct is brake with duty 0.

## How it fails closed

- `aegis_arbiter_decide()` is the only function that stores a non-zero duty. It does that only when the state is `RUNNING`, `fault_mask` is 0, and the PID candidate is finite.
- A tripped invariant latches `TRIPPED`. Commanded duty stays 0 on every later tick until reset.
- `RESET` returns to `ARMED`, never to `RUNNING`. A new `ARM` is required, and only while a heartbeat is still fresh.
- A non-finite sample, a redundant-state mismatch (`state` stored with its bitwise complement), or an unknown state latches a fault and forces duty 0.
- Telemetry uses a lock-free single-producer single-consumer ring. A full ring drops the sample and increments a counter. The loop does not wait.

Reset is a plain command. It is not authenticated.

## Fault to trip

Virtual time only (`sim-virtual`). Latency is `(trip_tick - onset_tick) * 5000` microseconds, from `results/sim/fault_trips.csv`.

Produced on an Apple M1 Pro (`sysctl -n machdep.cpu.brand_string`, Darwin arm64) by:

```bash
./build/aegis_lockstep --out results/sim/fault_trips.csv
```

That command exited 0 on 2026-09-30 and again on 2026-10-08 with identical results. Hardware latency is **not measured yet**.

| Scenario | Detected by | Configured check | sim-virtual latency | Hardware |
|---|---|---|---|---|
| nominal | none | no injected fault | no trip | not measured yet |
| heartbeat_drop | heartbeat | heartbeat older than 50000 µs | 50000 µs (onset 300, trip 310) | not measured yet |
| shaft_stall | stall | \|duty\| ≥ 0.60 and \|sum of encoder deltas\| ≤ 1 over 3 ticks | 15000 µs (onset 80, trip 83) | not measured yet |
| gyro_bias | encoder/IMU disagreement | \|w_enc − w_gyro\| > max(1.50, 0.25·max\|w\|) for 4 ticks | 15000 µs (onset 300, trip 303) | not measured yet |
| overspeed | velocity | \|w_hat\| > 12.0 rad/s for 1 tick | 0 µs (trip on onset tick 300) | not measured yet |
| tracking_step | tracking error | \|ref − theta\| > 0.20 rad for 2 ticks | 5000 µs (onset 300, trip 301) | not measured yet |

On the nominal run the largest \|ref − theta\| while `RUNNING` was 0.022264 rad. The overspeed row trips on the sample that already contains the injected 25 rad/s, so the virtual-time latency is 0. After that trip the injector keeps forcing plant speed, so the final angle in that row is not a stopped shaft. Commanded duty is still 0 (`motion_after_trip=0` on every row).

The campaign does not drain the ring. Each 500-tick scenario stored 64 samples and dropped 436.

## Cost per tick

Host wall clock, from `results/bench/step_timing.csv`. Produced on an Apple M1 Pro (Darwin arm64), Release build, on 2026-10-08 by:

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target aegis_bench
./build-release/aegis_bench --out results/bench/step_timing.csv
```

| Metric | Median | p99 | Worst batch |
|---|---|---|---|
| `aegis_step`, mean per step over a 500-step batch (4000 batches) | 22 ns | 26 ns | 142 ns |

Across four runs on this machine the median ranged from 20 to 22 ns and the worst batch from 120 to 216 ns. The workload is the nominal scenario's recorded inputs replayed open loop. The bench first checks that the replay reproduces the closed-loop trace exactly. Per-call timing is not reported because `CLOCK_MONOTONIC` on macOS ticks in 1 µs. This is a laptop under a general-purpose OS, not a microcontroller or an RTOS; hardware cost is **not measured yet**.

## Demo

The demo page runs the same C code in the browser. `demo/build.sh` compiles `core/`, the sim plant and `demo/demo_api.c` to WebAssembly with Emscripten. The loop is not re-implemented in JavaScript.

On the page you can arm the loop, move the target angle, and break it five ways: cut the heartbeat, jam the shaft, add a gyro bias, spin the shaft too fast, or jump the target. Each one trips a different check. The log shows how many ticks the trip took (virtual time, same as the table above).

```bash
./demo.sh
```

This builds, runs the tests and the fault campaign, and serves the page at http://localhost:8080. To only serve the page:

```bash
python3 -m http.server 8080 --directory demo
```

It has to be served over http. Opening `index.html` as a file does not load the `.wasm`.

`demo/aegis.js` and `demo/aegis.wasm` are build output but they are committed so the page can be hosted without a compiler. Rebuild them with `./demo/build.sh` (needs `brew install emscripten`) after changing any C file.

## Deploy (Railway)

The `Dockerfile` copies `demo/` into a Python image and serves it on `$PORT`. Point a Railway service at this repo. The image build has **not been tested locally yet**.

## Run

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/aegis_lockstep --out results/sim/fault_trips.csv --trace results/sim/traces
```

`ctest` runs two executables. `aegis_tests` is 18 Unity tests: each invariant, the arbiter property, SPSC order and drop, a pthread producer/consumer check (`received + dropped == produced`), deterministic replay, open-loop replay of recorded inputs, and no commanded motion after a trip. `aegis_demo_tests` is 11 tests of the demo glue in `demo/demo_api.c`: every injected fault trips with the expected fault, duty stays 0 after a trip, reset goes to `ARMED`, and arming needs a heartbeat.

## Limits

- One axis, one simplified 1-DOF plant. The gains are for that model, not a measured motor.
- Trip times above are virtual-time tick counts, not a hardware deadline and not a wall-clock measurement.
- No FreeRTOS, no microcontroller, no authenticated reset.
- The host campaign is not a real-time schedule. Do not read the drop counts as a logging-rate benchmark; the run fills the ring on purpose.
