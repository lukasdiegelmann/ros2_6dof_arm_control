#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_experiment.sh --outdir <dir> [--rmw <rmw_impl>] [--duration <sec>]

Examples:
  ./scripts/run_experiment.sh --outdir log/experiments/default
  ./scripts/run_experiment.sh --outdir log/experiments/cyclone --rmw rmw_cyclonedds_cpp
EOF
}

outdir=""
rmw=""
duration="75"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --outdir)
      outdir="$2"; shift 2 ;;
    --rmw)
      rmw="$2"; shift 2 ;;
    --duration)
      duration="$2"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$outdir" ]]; then
  echo "--outdir is required" >&2
  usage
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# Ensure ROS + overlay are available (arm_bringup must resolve).
# Note: ROS setup scripts are not always compatible with `set -u` (nounset).
set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
. install/setup.bash
set -u

if [[ -n "$rmw" ]]; then
  export RMW_IMPLEMENTATION="$rmw"
fi

mkdir -p "$outdir"

# Run diagnoser in background and a headless demo launch in foreground.
(timeout "${duration}s" stdbuf -oL -eL python3 scripts/diagnose_stalls.py \
  >"$outdir/diagnose.log" 2>&1) &

(timeout "${duration}s" stdbuf -oL -eL ros2 launch arm_bringup demo.launch.py \
  gz_headless:=-s gz_verbosity:=0 \
  >"$outdir/launch.log" 2>&1) || true

wait || true

log="$outdir/diagnose.log"

# Summary (grep -c returns exit code 1 when count is 0, so always || true)
echo "OUTDIR=$outdir"
echo -n "STALL_A="; (grep -c "STALL CLASS=A" "$log" || true)
echo -n "STALL_B="; (grep -c "STALL CLASS=B" "$log" || true)
echo -n "RESUMED="; (grep -c "RESUMED after" "$log" || true)
echo -n "CLOCK_BACK="; (grep -c "clock ROS time jumped BACKWARDS" "$log" || true)
echo -n "CLOCK_FWD="; (grep -c "clock ROS time jumped FORWARD" "$log" || true)
echo -n "JS_BACK="; (grep -c "joint_states stamp jumped BACKWARDS" "$log" || true)
echo -n "JS_FWD="; (grep -c "joint_states stamp jumped FORWARD" "$log" || true)
