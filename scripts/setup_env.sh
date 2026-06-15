#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-${ROOT_DIR}/.venv}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  printf 'Error: Python interpreter "%s" was not found.\n' "${PYTHON_BIN}" >&2
  exit 1
fi

printf 'Creating virtual environment at %s using %s\n' "${VENV_DIR}" "${PYTHON_BIN}"
"${PYTHON_BIN}" -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install -r "${ROOT_DIR}/requirements.txt"
"${VENV_DIR}/bin/python" -m pip install --editable "${ROOT_DIR}"

printf '\nEnvironment ready. Activate it with:\n'
printf '  source %s/bin/activate\n' "${VENV_DIR}"
