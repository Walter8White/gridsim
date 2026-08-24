#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAPTURE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/keyence_ljs8000/CPP/captures"
KEYENCE="$SCRIPT_DIR/keyence.sh"

TIMEOUT_MS=60000
SAVE_MODE="--save-all"

usage() {
  cat <<'EOF'
usage: scripts/keyence_test_setups.sh [--timeout-ms N] [--save-raw]

Runs several static KEYENCE scans with different optical settings.
Keep the target object fixed in the sensor field during the whole run.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout-ms)
      TIMEOUT_MS="$2"
      shift 2
      ;;
    --save-raw)
      SAVE_MODE="--save-raw"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

latest_meta() {
  find "$CAPTURE_DIR" -maxdepth 1 -name '*_meta.csv' -printf '%T@ %p\n' \
    | sort -n \
    | tail -1 \
    | cut -d' ' -f2-
}

meta_value() {
  local meta="$1"
  local key="$2"
  awk -F, -v key="$key" '$1 == key {print $2}' "$meta"
}

run_case() {
  local name="$1"
  local exposure="$2"
  local dynamic_range="$3"
  local light_mode="$4"
  local peak_filter="$5"

  echo
  echo "=== $name ==="
  "$KEYENCE" setting set exposure "$exposure"
  "$KEYENCE" setting set dynamic_range "$dynamic_range"
  "$KEYENCE" setting set light_mode "$light_mode"
  "$KEYENCE" setting set detection_sensitivity 5
  "$KEYENCE" setting set dead_zone_interpolation 2
  "$KEYENCE" setting set light_upper 99
  "$KEYENCE" setting set light_lower 99
  if [[ "$peak_filter" == "off" ]]; then
    "$KEYENCE" setting set peak_width_filter off
  else
    "$KEYENCE" setting set peak_width_filter on "$peak_filter"
  fi

  "$KEYENCE" scan "$SAVE_MODE" --timeout-ms "$TIMEOUT_MS"

  local meta
  meta="$(latest_meta)"
  echo "capture=$meta"
  echo "invalid_percent=$(meta_value "$meta" invalid_pixel_percent)"
  echo "height_min_raw=$(meta_value "$meta" height_min_raw)"
  echo "height_max_raw=$(meta_value "$meta" height_max_raw)"
}

run_case "balanced" 12 6 2 2
run_case "dark_surface_more_exposure" 14 8 0 off
run_case "dark_surface_max_exposure" 15 8 0 off
run_case "reflective_lower_exposure" 10 8 2 3
run_case "reflective_medium_exposure" 11 8 2 3

echo
echo "Done. Open the generated *_invalid_red.png files in:"
echo "$CAPTURE_DIR"
