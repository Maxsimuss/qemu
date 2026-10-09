#!/usr/bin/env bash
set -euo pipefail
fixture_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$fixture_dir/build"
"${ESP32_CC:-xtensa-esp32-elf-gcc}" -nostdlib \
    -Wl,-T,"$fixture_dir/counter.ld" -o "$fixture_dir/build/counter.elf" \
    "$fixture_dir/counter.S"
