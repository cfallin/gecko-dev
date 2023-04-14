#!/bin/bash
set -euxo pipefail

for x in octane/*.js; do
    cat $x | wizer --allow-wasi -r _start=wizer.resume -o $x.wasm obj-release/dist/bin/js
    ../weval/target/release/weval -i $x.wasm -o $x.wevaled.wasm
done
