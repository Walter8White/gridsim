#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CAPTURE_DIR="$REPO_ROOT/keyence_ljs8000/CPP/captures"
mkdir -p "$CAPTURE_DIR"
find "$CAPTURE_DIR" -mindepth 1 -maxdepth 1 -type f -delete
echo "Cleared $CAPTURE_DIR"
