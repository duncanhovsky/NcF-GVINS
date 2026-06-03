#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <scheme_name> <launch_file> [bag_path]" >&2
    exit 2
fi

SCHEME_NAME="$1"
LAUNCH_FILE="$2"
BAG_PATH="${3:-${I2NAV_BAG:-}}"
WORKSPACE="${NCF_GVINS_WS:-$HOME/ncf_gvins_ws}"
ROSLAUNCH_WAIT="${ROSLAUNCH_WAIT:-5}"
USE_IMAGE_REPUBLISH="${USE_IMAGE_REPUBLISH:-true}"
ENABLE_RVIZ="${ENABLE_RVIZ:-false}"
ROSBAG_ARGS="${ROSBAG_ARGS:-}"
OUTPUT_ROOT="${I2NAV_ABLATION_OUTPUT_ROOT:-/home/dyishere/ncf_gvins_ws/output/i2nav_ablation}"

if [[ ! -f "$WORKSPACE/devel/setup.bash" ]]; then
    echo "Cannot find $WORKSPACE/devel/setup.bash" >&2
    echo "Set NCF_GVINS_WS to your catkin workspace." >&2
    exit 1
fi

source "$WORKSPACE/devel/setup.bash"

echo "[i2Nav ablation] scheme=$SCHEME_NAME"
echo "[i2Nav ablation] launch=$LAUNCH_FILE"
mkdir -p "$OUTPUT_ROOT/$SCHEME_NAME"

if [[ -z "$BAG_PATH" ]]; then
    echo "[i2Nav ablation] no bag provided; starting roslaunch only."
    exec roslaunch ic_gvins "$LAUNCH_FILE" \
        use_image_republish:="$USE_IMAGE_REPUBLISH" \
        enable_rviz:="$ENABLE_RVIZ"
fi

if [[ ! -f "$BAG_PATH" ]]; then
    echo "Bag file does not exist: $BAG_PATH" >&2
    exit 1
fi

roslaunch ic_gvins "$LAUNCH_FILE" \
    use_image_republish:="$USE_IMAGE_REPUBLISH" \
    enable_rviz:="$ENABLE_RVIZ" &
ROSLAUNCH_PID=$!

cleanup() {
    if kill -0 "$ROSLAUNCH_PID" >/dev/null 2>&1; then
        kill "$ROSLAUNCH_PID" >/dev/null 2>&1 || true
        wait "$ROSLAUNCH_PID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

sleep "$ROSLAUNCH_WAIT"
echo "[i2Nav ablation] playing bag: $BAG_PATH"
# shellcheck disable=SC2086
rosbag play "$BAG_PATH" $ROSBAG_ARGS

cleanup
