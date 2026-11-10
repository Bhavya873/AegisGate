#!/usr/bin/env bash
# builds, runs the tests and the fault campaign, then serves the web demo
set -e
cd "$(dirname "$0")"

cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/aegis_lockstep --out results/sim/fault_trips.csv --trace results/sim/traces

echo "open http://localhost:8080 (Ctrl+C to stop)"
python3 -m http.server 8080 --directory demo
