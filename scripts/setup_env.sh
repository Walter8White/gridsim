#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${PYTHON_BIN:-}" ]]; then
  if [[ -x /usr/bin/python3 ]]; then
    PYTHON_BIN="/usr/bin/python3"
  else
    PYTHON_BIN="python3"
  fi
fi
VENV_DIR="${VENV_DIR:-${ROOT_DIR}/.venv}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  printf 'Error: Python interpreter "%s" was not found.\n' "${PYTHON_BIN}" >&2
  exit 1
fi

PYTHON_BIN="$(command -v "${PYTHON_BIN}")"
PYTHON_REAL="$(readlink -f "${PYTHON_BIN}")"

if [[ -x "${VENV_DIR}/bin/python" ]]; then
  VENV_PYTHON_REAL="$(readlink -f "${VENV_DIR}/bin/python")"
  if [[ "${VENV_PYTHON_REAL}" != "${PYTHON_REAL}" ]]; then
    printf 'Removing incompatible environment (%s, expected %s)\n' \
      "${VENV_PYTHON_REAL}" "${PYTHON_REAL}"
    rm -rf "${VENV_DIR}"
  elif ! "${VENV_DIR}/bin/python" -m pip --version >/dev/null 2>&1; then
    printf 'Removing incomplete environment at %s\n' "${VENV_DIR}"
    rm -rf "${VENV_DIR}"
  fi
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  printf 'Creating virtual environment at %s using %s\n' \
    "${VENV_DIR}" "${PYTHON_REAL}"
  if "${PYTHON_REAL}" -c "import ensurepip" >/dev/null 2>&1; then
    "${PYTHON_REAL}" -m venv "${VENV_DIR}"
  elif command -v uv >/dev/null 2>&1; then
    uv venv --python "${PYTHON_REAL}" --seed "${VENV_DIR}"
  else
    printf 'Error: %s cannot create a virtual environment because ensurepip is unavailable.\n' \
      "${PYTHON_REAL}" >&2
    printf 'Install it with: sudo apt install python3-venv\n' >&2
    exit 1
  fi
else
  printf 'Reusing virtual environment at %s\n' "${VENV_DIR}"
fi

# Prevent ROS or Conda Python paths from leaking into the standalone environment.
env -u PYTHONHOME -u PYTHONPATH \
  "${VENV_DIR}/bin/python" -m pip install --upgrade pip
env -u PYTHONHOME -u PYTHONPATH \
  "${VENV_DIR}/bin/python" -m pip install -r "${ROOT_DIR}/requirements.txt"
env -u PYTHONHOME -u PYTHONPATH \
  "${VENV_DIR}/bin/python" -m pip install --editable "${ROOT_DIR}"

printf '\nEnvironment ready. Activate it with:\n'
printf '  source %s/bin/activate\n' "${VENV_DIR}"
printf '  python --version\n'
