#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
KEYENCE_DIR="$REPO_ROOT/keyence_ljs8000/CPP"

usage() {
  cat <<'EOF'
usage: scripts/keyence.sh <command> [args]

Commands:
  status                 Check KEYENCE communication/status.
  scan [args]            Acquire a scan. Common args: --save-all --timeout-ms 60000
  setting [args]         Read/write settings. Example: setting get all
  cmd [args]             Send control command. Example: cmd laser_on
  ply [args]             Generate PLY mesh from latest capture.
  stl [args]             Generate STL mesh from latest capture.
  test-setups [args]     Run static scan tests with different optical settings.
  clear                  Delete files in KEYENCE captures folder.

Examples:
  scripts/keyence.sh status
  scripts/keyence.sh scan --save-all --timeout-ms 60000
  scripts/keyence.sh setting get all
  scripts/keyence.sh setting set exposure 14
  scripts/keyence.sh test-setups
  scripts/keyence.sh ply --stride 8
EOF
}

scan_usage() {
  cat <<'EOF'
usage: scripts/keyence.sh scan [--save-raw] [--save-invalid-image] [--save-all] [--timeout-ms N]

  --save-raw            Save height/luminance raw files and metadata CSV.
  --save-invalid-image  Save PNG image with invalid pixels in red.
  --save-all            Save raw files, metadata, and invalid-pixel image.
  --timeout-ms N        Acquisition timeout in milliseconds. Default: 30000.
EOF
}

cmd_usage() {
  cat <<'EOF'
usage: scripts/keyence.sh cmd laser_on|laser_off|clear_memory|trigger|raw <hex_command>
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

command="$1"
shift

case "$command" in
  status)
    cd "$KEYENCE_DIR"
    exec ./bin/check_status "$@"
    ;;
  scan)
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
      scan_usage
      exit 0
    fi
    cd "$KEYENCE_DIR"
    exec ./bin/main "$@"
    ;;
  setting)
    cd "$KEYENCE_DIR"
    exec ./bin/keyence_setting "$@"
    ;;
  cmd)
    if [[ $# -eq 0 || "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
      cmd_usage
      exit 0
    fi
    cd "$KEYENCE_DIR"
    exec ./bin/keyence_cmd "$@"
    ;;
  ply)
    exec python3 "$SCRIPT_DIR/keyence_generate_ply.py" "$@"
    ;;
  stl)
    exec python3 "$SCRIPT_DIR/keyence_generate_stl.py" "$@"
    ;;
  test-setups)
    exec "$SCRIPT_DIR/keyence_test_setups.sh" "$@"
    ;;
  clear)
    exec "$SCRIPT_DIR/keyence_clear_captures.sh" "$@"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown KEYENCE command: $command" >&2
    usage >&2
    exit 2
    ;;
esac
