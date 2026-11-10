#!/usr/bin/env bash
# builds, runs the tests, the fault campaign and the arm comparison, then serves the web demo
set -e
cd "$(dirname "$0")"

cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/aegis_lockstep --out results/sim/fault_trips.csv --trace results/sim/traces
./build/aegis_arm

echo "open http://localhost:8080 (Ctrl+C to stop)"
python3 -m http.server 8080 --directory demo
