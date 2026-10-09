#!/usr/bin/env bash
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
sim_root="$(cd -- "$project/../../../.." && pwd -P)"
if ! command -v idf.py >/dev/null 2>&1 && [[ -f "$sim_root/scripts/activate-idf.sh" ]]; then
    source "$sim_root/scripts/activate-idf.sh"
fi
command -v idf.py >/dev/null 2>&1 || {
    echo "idf.py is unavailable; activate an ESP-IDF environment first" >&2
    exit 1
}
export CCACHE_ENABLE="${CCACHE_ENABLE:-1}"
cd "$project"
rate="${APLL_DAC_RATE:-16000}"
port="${APLL_DAC_I2S_PORT:-0}"
idf.py --ccache -DIDF_TARGET=esp32 -DAPLL_DAC_RATE="$rate" \
  -DAPLL_DAC_I2S_PORT="$port" build 2>&1 | tee "$project/build.log"
idf.py --ccache merge-bin --output "$project/build/flash_image.bin" \
  --pad-to-size 4MB 2>&1 | tee "$project/merge.log"
