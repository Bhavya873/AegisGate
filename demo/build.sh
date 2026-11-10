#!/usr/bin/env bash
# compiles the C code to WebAssembly, needs emcc (brew install emscripten)
set -e
cd "$(dirname "$0")/.."

emcc -O2 -I core/include -I config -I sim/include -I demo \
    core/src/*.c sim/src/plant.c demo/demo_api.c -o demo/aegis.js
