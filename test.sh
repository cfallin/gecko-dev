#!/bin/bash

set -euo pipefail

./mach build --verbose
#echo 'function main() { print("hi"); }' | wizer --allow-wasi  -r _start=wizer.resume -o out.wasm obj-release/dist/bin/js
#wasmtime out.wasm

#cat fib.js | wizer --allow-wasi -r _start=wizer.resume -o out.wasm obj-release/dist/bin/js
#wasmtime out.wasm
cat count.js | wizer --allow-wasi -r _start=wizer.resume -o out.wasm obj-release/dist/bin/js
wasmtime out.wasm
