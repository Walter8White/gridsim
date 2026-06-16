#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-${HOME}/isaacsim/_build/linux-x86_64/release}"
ISAAC_PYTHON="${ISAAC_SIM_DIR}/python.sh"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  printf 'Error: Isaac Sim Python launcher not found at %s\n' "${ISAAC_PYTHON}" >&2
  printf 'Set ISAAC_SIM_DIR to the Isaac Sim release directory.\n' >&2
  exit 1
fi

export ISAAC_SIM_DIR
export WORKSPACE_DIR="${WORKSPACE_DIR:-${ROOT_DIR}/outputs/isaac}"

cd "${ROOT_DIR}"
exec "${ISAAC_PYTHON}" "${ROOT_DIR}/isaac/run_mvp.py" "$@"
