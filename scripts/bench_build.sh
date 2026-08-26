#!/usr/bin/env bash
# Measure clean vs incremental colcon build times for the sim workspace.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/humble/setup.bash
set -u

JOBS="$(nproc)"
CMAKE_ARGS=(--cmake-args -DCMAKE_BUILD_TYPE=Release)

fmt_secs() {
  awk -v s="$1" 'BEGIN { printf "%.1fs (%.1fm)", s, s/60 }'
}

run_timed() {
  local label="$1"
  shift
  local start end elapsed
  start=$(date +%s.%N)
  "$@"
  end=$(date +%s.%N)
  elapsed=$(awk -v a="$start" -v b="$end" 'BEGIN { printf "%.2f", b - a }')
  echo "$label|$elapsed"
}

verify_stonefish_prefix() {
  local cache="$ROOT/build/stonefish_ros2/CMakeCache.txt"
  if [[ ! -f "$cache" ]]; then
    echo "warning: $cache missing; cannot verify Stonefish prefix" >&2
    return 0
  fi
  local found
  found=$(grep -m1 '^Stonefish_DIR:' "$cache" | awk '{print $2}')
  echo "stonefish_ros2 Stonefish_DIR=$found"
  case "$found" in
    "$ROOT/install"/*) return 0 ;;
    *) echo "error: stonefish_ros2 did not use workspace install/" >&2; return 1 ;;
  esac
}

echo "=== autonomy-stack colcon benchmark ==="
echo "date: $(date -Is)"
echo "root: $ROOT"
echo "jobs: $JOBS"
echo "cpu: $(lscpu | awk -F: '/Model name/{print $2}' | xargs)"
echo

# --- 1) Colcon, fully clean (no cargo cache) ---
rm -rf build install log target vendor/stonefish/build vendor/stonefish/install
COLCON_CLEAN_NO_CACHE=$(run_timed colcon_clean_no_cache colcon build "${CMAKE_ARGS[@]}")
verify_stonefish_prefix
echo "colcon build (clean, no cargo cache): $(fmt_secs "${COLCON_CLEAN_NO_CACHE#*|}")"

# --- 2) Colcon, clean workspace but warm cargo cache ---
rm -rf build install log
COLCON_CLEAN_WARM_CACHE=$(run_timed colcon_clean_warm_cache colcon build "${CMAKE_ARGS[@]}")
verify_stonefish_prefix
echo "colcon build (clean ws, warm cargo): $(fmt_secs "${COLCON_CLEAN_WARM_CACHE#*|}")"

# --- 3) Incremental: no source changes ---
COLCON_NOOP=$(run_timed colcon_noop colcon build "${CMAKE_ARGS[@]}")
echo "colcon build (rebuild, no changes): $(fmt_secs "${COLCON_NOOP#*|}")"

# --- 4) Incremental: touch one file per package type ---
touch src/bringup/launch/stonefish.launch.py
touch src/detection_mocker/src/main.cpp
touch src/mission_executor/src/main.rs
COLCON_TOUCH=$(run_timed colcon_touch colcon build "${CMAKE_ARGS[@]}")
echo "colcon build (touch launch+cpp+rust): $(fmt_secs "${COLCON_TOUCH#*|}")"

echo
echo "=== artifact sizes ==="
du -sh build install target 2>/dev/null | sed 's/^/  /'

echo
echo "=== summary (seconds) ==="
printf '%s\n' \
  "$COLCON_CLEAN_NO_CACHE" \
  "$COLCON_CLEAN_WARM_CACHE" \
  "$COLCON_NOOP" \
  "$COLCON_TOUCH" \
  | column -t -s'|'
