#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/_run_i2nav_ablation_common.sh" \
    "06_nc_local_bootstrap" \
    "i2nav_ablation_06_nc_local_bootstrap.launch" \
    "${1:-${I2NAV_BAG:-}}"
