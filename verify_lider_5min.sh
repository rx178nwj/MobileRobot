#!/usr/bin/env bash
set -euo pipefail

# 5-minute hardware verification for LiderModule spiral-scan sync.
# Steps:
#  1) Optional interference stop
#  2) Firmware compile/upload (optional skip)
#  3) Host scan run
#  4) PASS/FAIL check:
#     - at least one "tilt=a->b" slice log
#     - scan complete N/N (expected step count)

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="${ROOT_DIR}/LiderModule/firmware/lidar_tilt_3d"
HOST_SCRIPT="${ROOT_DIR}/LiderModule/host/rpi_tilt_3d.py"

PORT="/dev/ttyACM0"
FQBN="esp32:esp32:XIAO_ESP32C3:CDCOnBoot=default,UploadSpeed=921600"
TILT_MIN="-45"
TILT_MAX="45"
STEP="2"
SCAN_TIMEOUT="15"
RUN_TIMEOUT_SEC="240"
STOP_INTERFERENCE="0"
SKIP_FLASH="0"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/lider_verify_5min_${STAMP}"
RUN_LOG="${LOG_DIR}/scan.log"

info() { printf '[INFO] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*" >&2; }
err()  { printf '[ERROR] %s\n' "$*" >&2; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --port PATH              Serial port (default: ${PORT})
  --fqbn FQBN              Board FQBN (default: ${FQBN})
  --tilt-min DEG           Scan tilt min (default: ${TILT_MIN})
  --tilt-max DEG           Scan tilt max (default: ${TILT_MAX})
  --step DEG               Scan step (default: ${STEP})
  --scan-timeout SEC       Host receive timeout (default: ${SCAN_TIMEOUT})
  --run-timeout SEC        Whole scan process timeout (default: ${RUN_TIMEOUT_SEC})
  --log-dir DIR            Log output directory (default: ${LOG_DIR})
  --stop-interference      Try stopping known competing processes/containers
  --skip-flash             Skip firmware compile/upload
  -h, --help               Show this help

Examples:
  $(basename "$0")
  $(basename "$0") --port /dev/ttyACM0 --stop-interference
  $(basename "$0") --skip-flash --tilt-min -45 --tilt-max 45 --step 2
EOF
}

require_cmd() {
  local c="$1"
  if ! command -v "$c" >/dev/null 2>&1; then
    err "required command not found: ${c}"
    exit 1
  fi
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --port) PORT="${2:?}"; shift 2 ;;
      --fqbn) FQBN="${2:?}"; shift 2 ;;
      --tilt-min) TILT_MIN="${2:?}"; shift 2 ;;
      --tilt-max) TILT_MAX="${2:?}"; shift 2 ;;
      --step) STEP="${2:?}"; shift 2 ;;
      --scan-timeout) SCAN_TIMEOUT="${2:?}"; shift 2 ;;
      --run-timeout) RUN_TIMEOUT_SEC="${2:?}"; shift 2 ;;
      --log-dir)
        LOG_DIR="${2:?}"
        RUN_LOG="${LOG_DIR}/scan.log"
        shift 2
        ;;
      --stop-interference) STOP_INTERFERENCE="1"; shift ;;
      --skip-flash) SKIP_FLASH="1"; shift ;;
      -h|--help) usage; exit 0 ;;
      *)
        err "unknown option: $1"
        usage
        exit 2
        ;;
    esac
  done
}

validate_inputs() {
  if [[ ! -d "${SKETCH_DIR}" ]]; then
    err "sketch directory not found: ${SKETCH_DIR}"
    exit 1
  fi
  if [[ ! -f "${HOST_SCRIPT}" ]]; then
    err "host script not found: ${HOST_SCRIPT}"
    exit 1
  fi
  if [[ ! -e "${PORT}" ]]; then
    err "serial port not found: ${PORT}"
    exit 1
  fi
}

show_port_users() {
  if command -v lsof >/dev/null 2>&1; then
    lsof "${PORT}" || true
  elif command -v fuser >/dev/null 2>&1; then
    fuser -v "${PORT}" || true
  else
    warn "lsof/fuser not available; cannot inspect port users"
  fi
}

stop_known_interference() {
  info "stopping known competing processes (best effort)"
  if command -v sudo >/dev/null 2>&1; then
    sudo -n pkill -f "lider_module_node|ws_edge_bringup.launch.py|rpi_tilt_3d.py --port ${PORT}" >/dev/null 2>&1 || true
  else
    pkill -f "lider_module_node|ws_edge_bringup.launch.py|rpi_tilt_3d.py --port ${PORT}" >/dev/null 2>&1 || true
  fi

  if command -v docker >/dev/null 2>&1; then
    local cids
    cids="$(docker ps --filter "ancestor=mobile-robot-ros2:jazzy" -q || true)"
    if [[ -n "${cids}" ]]; then
      info "stopping ros2 containers: ${cids}"
      docker stop ${cids} >/dev/null || true
    fi
  fi
}

compile_and_upload() {
  info "compiling firmware"
  arduino-cli compile --fqbn "${FQBN}" "${SKETCH_DIR}"

  info "uploading firmware to ${PORT}"
  arduino-cli upload -p "${PORT}" --fqbn "${FQBN}" "${SKETCH_DIR}"
}

run_scan() {
  local expected_steps
  expected_steps="$(
    awk -v min="${TILT_MIN}" -v max="${TILT_MAX}" -v step="${STEP}" '
      BEGIN{
        if (step <= 0) { print 0; exit }
        if (max < min) { t=min; min=max; max=t }
        print int(((max-min)/step)+0.5)+1
      }'
  )"

  info "running host scan (expected steps: ${expected_steps})"
  set +e
  timeout "${RUN_TIMEOUT_SEC}s" \
    python3 "${HOST_SCRIPT}" \
      --port "${PORT}" \
      --linear-scan \
      --tilt-min "${TILT_MIN}" \
      --tilt-max "${TILT_MAX}" \
      --step "${STEP}" \
      --no-viz \
      --timeout "${SCAN_TIMEOUT}" \
      2>&1 | tee "${RUN_LOG}"
  local rc=${PIPESTATUS[0]}
  set -e

  if [[ ${rc} -eq 124 ]]; then
    err "scan command timed out (${RUN_TIMEOUT_SEC}s)"
    return 1
  fi
  if [[ ${rc} -ne 0 ]]; then
    err "scan command failed with exit code ${rc}"
    return 1
  fi

  local spiral_count
  spiral_count="$(grep -Ec '\[slice\] tilt=[-0-9.]+->[-0-9.]+ deg' "${RUN_LOG}" || true)"
  local complete_line
  complete_line="$(grep -E '\[scan\] complete: [0-9]+/[0-9]+' "${RUN_LOG}" | tail -n 1 || true)"
  local expected_complete="[scan] complete: ${expected_steps}/${expected_steps}"

  info "spiral slice lines: ${spiral_count}"
  info "last complete line: ${complete_line:-<none>}"

  if [[ "${spiral_count}" -lt 1 ]]; then
    err "no spiral sync line detected (tilt=a->b)"
    return 1
  fi
  if [[ "${complete_line}" != "${expected_complete}" ]]; then
    err "unexpected completion. expected '${expected_complete}'"
    return 1
  fi

  info "verification PASS"
  return 0
}

main() {
  parse_args "$@"
  mkdir -p "${LOG_DIR}"

  require_cmd arduino-cli
  require_cmd python3
  require_cmd timeout
  validate_inputs

  info "log directory: ${LOG_DIR}"
  info "serial users before run:"
  show_port_users

  if [[ "${STOP_INTERFERENCE}" == "1" ]]; then
    stop_known_interference
    info "serial users after stop attempt:"
    show_port_users
  fi

  if [[ "${SKIP_FLASH}" == "0" ]]; then
    compile_and_upload
  else
    info "skipping firmware compile/upload (--skip-flash)"
  fi

  run_scan

  cat <<EOF

[RESULT] PASS
  - Firmware: $( [[ "${SKIP_FLASH}" == "0" ]] && echo "uploaded" || echo "skipped" )
  - Spiral sync: detected (tilt=a->b)
  - Completion: expected step count reached
  - Scan log: ${RUN_LOG}
EOF
}

main "$@"
