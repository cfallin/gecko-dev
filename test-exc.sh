#!/bin/bash
./mach build
cat exc.js| wizer --allow-wasi -r _start=wizer.resume -o exc.wasm obj-release/dist/bin/js
wasmtime exc.js
