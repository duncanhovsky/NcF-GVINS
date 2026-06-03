#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCHEME_SCRIPT_DIR="$SCRIPT_DIR/i2nav_ablation"
CONFIG_DIR="$REPO_ROOT/config/i2nav_ablation"
LAUNCH_DIR="$REPO_ROOT/ic_gvins/launch"

WORKSPACE="${NCF_GVINS_WS:-$HOME/ncf_gvins_ws}"
OUTPUT_ROOT="${I2NAV_ABLATION_BATCH_OUTPUT_ROOT:-/home/dyishere/ncf_gvins_ws/output/i2nav_ablation_batch}"
BATCH_ID="${I2NAV_ABLATION_BATCH_ID:-$(date +%Y%m%d_%H%M%S)}"
REPEAT_COUNT="${I2NAV_ABLATION_REPEAT_COUNT:-10}"
ROSLAUNCH_WAIT="${ROSLAUNCH_WAIT:-5}"
USE_IMAGE_REPUBLISH="${USE_IMAGE_REPUBLISH:-true}"
ENABLE_RVIZ="${ENABLE_RVIZ:-false}"
ROSBAG_ARGS="${ROSBAG_ARGS:-}"
CONTINUE_ON_ERROR="${I2NAV_ABLATION_CONTINUE_ON_ERROR:-false}"
DRY_RUN=false

DATASETS=(
    "building00|/media/dyishere/TOSHIBA EXT/Dataset/i2Nav-Robot/building00/building00_processed.bag"
    "building01|/media/dyishere/TOSHIBA EXT/Dataset/i2Nav-Robot/building01/building01_processed.bag"
    "building02|/media/dyishere/TOSHIBA EXT/Dataset/i2Nav-Robot/building02/building02_processed.bag"
    "street00|/media/dyishere/新加卷/Dataset/i2Nav-Robot/street00/street00_processed.bag"
    "street01|/media/dyishere/新加卷/Dataset/i2Nav-Robot/street01/street01_processed.bag"
    "street02|/media/dyishere/新加卷/Dataset/i2Nav-Robot/street02/street02_processed.bag"
    "playground00|/media/dyishere/RED/dataset/playground00_processed.orig.bag"
    "parking00|/media/dyishere/新加卷/Dataset/i2Nav-Robot/parking00/parking00_processed.bag"
)

usage() {
    cat <<USAGE
Usage: $(basename "$0") [options]

Run every i2Nav ablation scheme against every configured bag sequence N times.

Options:
  --repeat N              Repeat count per scheme/sequence pair (default: $REPEAT_COUNT)
  --output-root DIR       Batch output root (default: $OUTPUT_ROOT)
  --batch-id ID           Batch directory name (default: timestamp)
  --continue-on-error     Continue after one run fails
  --dry-run               Print the run matrix without starting ROS
  -h, --help              Show this help

Environment:
  NCF_GVINS_WS                       Catkin workspace (default: $HOME/ncf_gvins_ws)
  I2NAV_ABLATION_BATCH_OUTPUT_ROOT   Same as --output-root
  I2NAV_ABLATION_BATCH_ID            Same as --batch-id
  I2NAV_ABLATION_REPEAT_COUNT        Same as --repeat
  I2NAV_ABLATION_CONTINUE_ON_ERROR   true/false
  ROSLAUNCH_WAIT                     Seconds to wait before rosbag play (default: 5)
  USE_IMAGE_REPUBLISH                true/false passed to launch (default: true)
  ENABLE_RVIZ                        true/false passed to launch (default: false)
  ROSBAG_ARGS                        Extra args passed to rosbag play
USAGE
}

die() {
    echo "[i2Nav batch] ERROR: $*" >&2
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --repeat)
            [[ $# -ge 2 ]] || die "--repeat requires a value"
            REPEAT_COUNT="$2"
            shift 2
            ;;
        --output-root)
            [[ $# -ge 2 ]] || die "--output-root requires a value"
            OUTPUT_ROOT="$2"
            shift 2
            ;;
        --batch-id)
            [[ $# -ge 2 ]] || die "--batch-id requires a value"
            BATCH_ID="$2"
            shift 2
            ;;
        --continue-on-error)
            CONTINUE_ON_ERROR=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
done

[[ "$REPEAT_COUNT" =~ ^[0-9]+$ ]] || die "--repeat must be a positive integer"
(( REPEAT_COUNT > 0 )) || die "--repeat must be greater than 0"
case "$CONTINUE_ON_ERROR" in
    true|false) ;;
    *) die "I2NAV_ABLATION_CONTINUE_ON_ERROR must be true or false" ;;
esac

RUN_ROOT="$OUTPUT_ROOT/$BATCH_ID"

mapfile -t RUN_SCRIPTS < <(find "$SCHEME_SCRIPT_DIR" -maxdepth 1 -type f -name 'run_[0-9][0-9]_*.sh' | sort)
(( ${#RUN_SCRIPTS[@]} > 0 )) || die "no run_XX_*.sh scripts found in $SCHEME_SCRIPT_DIR"

enable_nc_reloc_from_launch() {
    local launch_file="$1"
    local value
    value="$(sed -n 's/.*name="enable_nc_reloc" value="\([^"]*\)".*/\1/p' "$launch_file" | head -n 1)"
    case "$value" in
        true|false)
            printf '%s\n' "$value"
            ;;
        *)
            die "cannot parse enable_nc_reloc from $launch_file"
            ;;
    esac
}

SCHEMES=()
for run_script in "${RUN_SCRIPTS[@]}"; do
    run_name="$(basename "$run_script" .sh)"
    scheme="${run_name#run_}"
    config_file="$CONFIG_DIR/$scheme.yaml"
    launch_file="$LAUNCH_DIR/i2nav_ablation_$scheme.launch"

    [[ -f "$config_file" ]] || die "missing config for $scheme: $config_file"
    [[ -f "$launch_file" ]] || die "missing launch for $scheme: $launch_file"

    enable_nc_reloc="$(enable_nc_reloc_from_launch "$launch_file")"
    SCHEMES+=("$scheme|$run_script|$config_file|$launch_file|$enable_nc_reloc")
done

TOTAL_RUNS=$(( ${#SCHEMES[@]} * ${#DATASETS[@]} * REPEAT_COUNT ))

print_plan() {
    local current=0
    echo "[i2Nav batch] schemes: ${#SCHEMES[@]}"
    echo "[i2Nav batch] sequences: ${#DATASETS[@]}"
    echo "[i2Nav batch] repeat: $REPEAT_COUNT"
    echo "[i2Nav batch] total runs: $TOTAL_RUNS"
    echo "[i2Nav batch] output root: $RUN_ROOT"

    for scheme_entry in "${SCHEMES[@]}"; do
        IFS='|' read -r scheme _run_script _config_file _launch_file _enable_nc_reloc <<< "$scheme_entry"
        for dataset_entry in "${DATASETS[@]}"; do
            IFS='|' read -r sequence bag_path <<< "$dataset_entry"
            for repeat in $(seq 1 "$REPEAT_COUNT"); do
                current=$((current + 1))
                printf -v repeat_name "run_%02d" "$repeat"
                echo "[$current/$TOTAL_RUNS] $scheme / $sequence / $repeat_name"
                echo "  bag: $bag_path"
                echo "  dir: $RUN_ROOT/$scheme/$sequence/$repeat_name"
            done
        done
    done
}

if [[ "$DRY_RUN" == true ]]; then
    print_plan
    exit 0
fi

[[ -f "$WORKSPACE/devel/setup.bash" ]] || die "cannot find $WORKSPACE/devel/setup.bash; set NCF_GVINS_WS"

for dataset_entry in "${DATASETS[@]}"; do
    IFS='|' read -r _sequence bag_path <<< "$dataset_entry"
    [[ -f "$bag_path" ]] || die "bag file does not exist: $bag_path"
done

# shellcheck source=/dev/null
source "$WORKSPACE/devel/setup.bash"

ROSLAUNCH_PID=""

cleanup_roslaunch() {
    if [[ -n "${ROSLAUNCH_PID:-}" ]] && kill -0 "$ROSLAUNCH_PID" >/dev/null 2>&1; then
        kill "$ROSLAUNCH_PID" >/dev/null 2>&1 || true
        wait "$ROSLAUNCH_PID" >/dev/null 2>&1 || true
    fi
    ROSLAUNCH_PID=""
}

trap cleanup_roslaunch EXIT
trap 'echo "[i2Nav batch] interrupted"; cleanup_roslaunch; exit 130' INT TERM

escape_sed_replacement() {
    printf '%s' "$1" | sed -e 's/[\/&|]/\\&/g'
}

write_run_config() {
    local source_config="$1"
    local target_config="$2"
    local output_path="$3"
    local escaped_output

    escaped_output="$(escape_sed_replacement "$output_path")"
    sed \
        -e "s|^outputpath:.*|outputpath: \"$escaped_output\"|" \
        -e 's|^is_make_outputdir:.*|is_make_outputdir: false|' \
        "$source_config" > "$target_config"

    grep -q '^outputpath: ' "$target_config" || die "failed to write outputpath into $target_config"
    grep -q '^is_make_outputdir: false' "$target_config" || die "failed to disable timestamp output dir in $target_config"
}

write_run_launch() {
    local target_launch="$1"
    local run_config="$2"
    local enable_nc_reloc="$3"

    cat > "$target_launch" <<EOF
<launch>
    <arg name="use_image_republish" default="true"/>
    <arg name="enable_rviz" default="false"/>
    <include file="\$(find ic_gvins)/launch/i2nav_ablation_common.launch">
        <arg name="configfile" value="$run_config"/>
        <arg name="enable_nc_reloc" value="$enable_nc_reloc"/>
        <arg name="use_image_republish" value="\$(arg use_image_republish)"/>
        <arg name="enable_rviz" value="\$(arg enable_rviz)"/>
    </include>
</launch>
EOF
}

run_one() {
    local ordinal="$1"
    local scheme="$2"
    local run_script="$3"
    local source_config="$4"
    local source_launch="$5"
    local enable_nc_reloc="$6"
    local sequence="$7"
    local bag_path="$8"
    local repeat="$9"
    local repeat_name
    local run_dir
    local run_config
    local run_launch
    local run_log
    local status_file
    local start_time
    local end_time
    local bag_status
    local launch_status
    local roslaunch_alive_after_bag
    local roslaunch_status_after_bag

    printf -v repeat_name "run_%02d" "$repeat"
    run_dir="$RUN_ROOT/$scheme/$sequence/$repeat_name"
    run_config="$run_dir/config.yaml"
    run_launch="$run_dir/launch.launch"
    run_log="$run_dir/run.log"
    status_file="$run_dir/status.txt"

    [[ ! -e "$run_dir" ]] || die "run directory already exists, refusing to overwrite: $run_dir"
    mkdir -p "$run_dir"

    write_run_config "$source_config" "$run_config" "$run_dir"
    write_run_launch "$run_launch" "$run_config" "$enable_nc_reloc"

    start_time="$(date --iso-8601=seconds)"
    {
        echo "scheme=$scheme"
        echo "sequence=$sequence"
        echo "repeat=$repeat"
        echo "bag=$bag_path"
        echo "source_run_script=$run_script"
        echo "source_config=$source_config"
        echo "source_launch=$source_launch"
        echo "enable_nc_reloc=$enable_nc_reloc"
        echo "run_dir=$run_dir"
        echo "started_at=$start_time"
    } > "$run_dir/metadata.txt"

    {
        echo "status=running"
        echo "started_at=$start_time"
    } > "$status_file"

    echo "[i2Nav batch] [$ordinal/$TOTAL_RUNS] $scheme / $sequence / $repeat_name"
    echo "[i2Nav batch] output: $run_dir"

    roslaunch "$run_launch" \
        use_image_republish:="$USE_IMAGE_REPUBLISH" \
        enable_rviz:="$ENABLE_RVIZ" > "$run_log" 2>&1 &
    ROSLAUNCH_PID=$!

    sleep "$ROSLAUNCH_WAIT"

    if ! kill -0 "$ROSLAUNCH_PID" >/dev/null 2>&1; then
        set +e
        wait "$ROSLAUNCH_PID"
        launch_status=$?
        set -e
        end_time="$(date --iso-8601=seconds)"
        {
            echo "status=failed"
            echo "reason=roslaunch_exited_before_rosbag"
            echo "roslaunch_status=$launch_status"
            echo "started_at=$start_time"
            echo "ended_at=$end_time"
        } > "$status_file"
        ROSLAUNCH_PID=""
        return 1
    fi

    echo "[i2Nav batch] rosbag play: $bag_path" >> "$run_log"
    set +e
    # shellcheck disable=SC2086
    rosbag play "$bag_path" $ROSBAG_ARGS >> "$run_log" 2>&1
    bag_status=$?
    set -e

    roslaunch_alive_after_bag=true
    roslaunch_status_after_bag=0
    if kill -0 "$ROSLAUNCH_PID" >/dev/null 2>&1; then
        cleanup_roslaunch
    else
        roslaunch_alive_after_bag=false
        set +e
        wait "$ROSLAUNCH_PID"
        roslaunch_status_after_bag=$?
        set -e
        ROSLAUNCH_PID=""
    fi

    end_time="$(date --iso-8601=seconds)"
    if (( bag_status == 0 )) && [[ "$roslaunch_alive_after_bag" == true ]]; then
        {
            echo "status=success"
            echo "rosbag_status=$bag_status"
            echo "started_at=$start_time"
            echo "ended_at=$end_time"
        } > "$status_file"
        return 0
    fi

    if (( bag_status != 0 )); then
        {
            echo "status=failed"
            echo "reason=rosbag_play_failed"
            echo "rosbag_status=$bag_status"
            echo "started_at=$start_time"
            echo "ended_at=$end_time"
        } > "$status_file"
    else
        {
            echo "status=failed"
            echo "reason=roslaunch_exited_during_rosbag"
            echo "rosbag_status=$bag_status"
            echo "roslaunch_status=$roslaunch_status_after_bag"
            echo "started_at=$start_time"
            echo "ended_at=$end_time"
        } > "$status_file"
    fi

    return 1
}

mkdir -p "$RUN_ROOT"

echo "[i2Nav batch] starting"
echo "[i2Nav batch] output root: $RUN_ROOT"
echo "[i2Nav batch] total runs: $TOTAL_RUNS"

current=0
failures=0
for scheme_entry in "${SCHEMES[@]}"; do
    IFS='|' read -r scheme run_script source_config source_launch enable_nc_reloc <<< "$scheme_entry"
    for dataset_entry in "${DATASETS[@]}"; do
        IFS='|' read -r sequence bag_path <<< "$dataset_entry"
        for repeat in $(seq 1 "$REPEAT_COUNT"); do
            current=$((current + 1))
            if run_one "$current" "$scheme" "$run_script" "$source_config" "$source_launch" \
                "$enable_nc_reloc" "$sequence" "$bag_path" "$repeat"; then
                :
            else
                failures=$((failures + 1))
                echo "[i2Nav batch] run failed; see $RUN_ROOT/$scheme/$sequence/run_$(printf '%02d' "$repeat")/run.log" >&2
                if [[ "$CONTINUE_ON_ERROR" != true ]]; then
                    echo "[i2Nav batch] stopping after first failure; pass --continue-on-error to keep going" >&2
                    exit 1
                fi
            fi
        done
    done
done

if (( failures > 0 )); then
    echo "[i2Nav batch] finished with $failures failure(s)" >&2
    exit 1
fi

echo "[i2Nav batch] finished all $TOTAL_RUNS runs"
