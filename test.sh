#!/bin/bash

set -euo pipefail

./mach build --verbose
echo 'function main() { print("hi"); }' | wizer --allow-wasi  -r _start=wizer.resume -o out.wasm obj-debug/dist/bin/js
wasmtime out.wasm
