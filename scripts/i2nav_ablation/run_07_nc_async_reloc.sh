#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/_run_i2nav_ablation_common.sh" \
    "07_nc_async_reloc" \
    "i2nav_ablation_07_nc_async_reloc.launch" \
    "${1:-${I2NAV_BAG:-}}"
