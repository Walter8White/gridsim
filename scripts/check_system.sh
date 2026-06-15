#!/usr/bin/env bash
set -u

printf 'gridsim system check\n'
printf '%s\n' '--------------------'

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  printf 'Ubuntu: %s %s\n' "${NAME:-unknown}" "${VERSION_ID:-unknown}"
else
  printf 'Ubuntu: not detectable\n'
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_NAME="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n 1)"
  if [[ -n "${GPU_NAME}" ]]; then
    printf 'NVIDIA GPU: available (%s)\n' "${GPU_NAME}"
  else
    printf 'NVIDIA GPU: nvidia-smi found, but the driver/GPU is unavailable\n'
  fi
else
  printf 'NVIDIA GPU: nvidia-smi not found\n'
fi

if command -v python3 >/dev/null 2>&1; then
  printf 'Python: %s (%s)\n' "$(python3 --version 2>&1)" "$(command -v python3)"
else
  printf 'Python: python3 not found\n'
fi

if command -v ros2 >/dev/null 2>&1; then
  printf 'ROS 2: available (%s)\n' "$(command -v ros2)"
elif [[ -r /opt/ros/jazzy/setup.bash ]]; then
  printf 'ROS 2: Jazzy detected at /opt/ros/jazzy; source setup.bash to use it\n'
else
  printf 'ROS 2: not detected\n'
fi

ISAAC_PATH=""
ISAAC_LAUNCHER=""
for candidate in \
  "${ISAAC_SIM_PATH:-}" \
  "${HOME}/isaacsim" \
  "${HOME}/.local/share/ov/pkg/isaac-sim"* \
  "/opt/isaacsim"; do
  if [[ -n "${candidate}" && -e "${candidate}" ]]; then
    ISAAC_PATH="${candidate}"
    for launcher in \
      "${candidate}/isaac-sim.sh" \
      "${candidate}/_build/linux-x86_64/release/isaac-sim.sh"; do
      if [[ -x "${launcher}" ]]; then
        ISAAC_LAUNCHER="${launcher}"
        break
      fi
    done
    break
  fi
done

if command -v isaac-sim.sh >/dev/null 2>&1; then
  printf 'Isaac Sim: available (%s)\n' "$(command -v isaac-sim.sh)"
elif command -v isaacsim >/dev/null 2>&1; then
  printf 'Isaac Sim: available (%s)\n' "$(command -v isaacsim)"
elif [[ -n "${ISAAC_LAUNCHER}" ]]; then
  ISAAC_VERSION_FILE="${ISAAC_PATH}/VERSION"
  ISAAC_VERSION="unknown version"
  if [[ -r "${ISAAC_VERSION_FILE}" ]]; then
    ISAAC_VERSION="$(tr -d '\n' < "${ISAAC_VERSION_FILE}")"
  fi
  printf 'Isaac Sim: available (%s, %s)\n' "${ISAAC_VERSION}" "${ISAAC_LAUNCHER}"
elif [[ -n "${ISAAC_PATH}" ]]; then
  printf 'Isaac Sim: possible installation detected at %s\n' "${ISAAC_PATH}"
else
  printf 'Isaac Sim: not detected; install it separately\n'
fi
