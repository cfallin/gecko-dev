#!/bin/bash
set -e -x -o

./mach build
cat markdown.js| wizer --allow-wasi -r _start=wizer.resume -o markdown.wasm obj-release/dist/bin/js
../weval/target/release/weval -i markdown.wasm -o markdown-wevaled.wasm
