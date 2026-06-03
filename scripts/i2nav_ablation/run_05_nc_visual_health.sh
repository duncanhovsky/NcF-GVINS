#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/_run_i2nav_ablation_common.sh" \
    "05_nc_visual_health" \
    "i2nav_ablation_05_nc_visual_health.launch" \
    "${1:-${I2NAV_BAG:-}}"
