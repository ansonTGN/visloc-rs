#!/usr/bin/env bash
set -Eeuo pipefail

# Headless upstream Basalt EuRoC run. Evaluation is deliberately separate:
# this command never passes the EuRoC ground-truth path to basalt_vio.

UPSTREAM_SHA="0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
ORACLE_ROOT="${ORACLE_ROOT:-${HOME}/visloc-basalt-oracle-${UPSTREAM_SHA:0:12}}"
EUROC_ROOT="${EUROC_ROOT:-/mnt/e/datasets/euroc_mav}"
RESULT_ROOT="${RESULT_ROOT:-${HOME}/basalt-oracle-results}"
BUILD_DIR="${ORACLE_BUILD_DIR:-${ORACLE_ROOT}/build/relwithdebinfo}"
SEQUENCE="${1:-MH_01_easy}"
RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
MAX_FRAMES="${MAX_FRAMES:-0}"
NUM_THREADS="${NUM_THREADS:-}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

[[ "$(uname -s)" == "Linux" ]] || die "run this script inside Ubuntu-22.04 WSL2"
[[ "$(git -C "${ORACLE_ROOT}" rev-parse HEAD 2>/dev/null)" == "${UPSTREAM_SHA}" ]] \
  || die "oracle checkout is missing or not at ${UPSTREAM_SHA}; run setup_oracle_wsl.sh"
[[ -x "${BUILD_DIR}/basalt_vio" ]] \
  || die "basalt_vio is missing at ${BUILD_DIR}; run setup_oracle_wsl.sh or set ORACLE_BUILD_DIR"

DATASET="${EUROC_ROOT}/${SEQUENCE}"
if [[ ! -d "${DATASET}/mav0" && -d "${EUROC_ROOT}/machine_hall/${SEQUENCE}/mav0" ]]; then
  DATASET="${EUROC_ROOT}/machine_hall/${SEQUENCE}"
fi
[[ -d "${DATASET}/mav0" ]] \
  || die "missing EuRoC sequence: ${DATASET}/mav0"

OUT="${RESULT_ROOT}/${SEQUENCE}/${RUN_ID}"
mkdir -p "${OUT}"
mkdir -p "${OUT}/marg_data"

CONFIG="${ORACLE_ROOT}/data/euroc_config.json"
CALIB="${ORACLE_ROOT}/data/euroc_ds_calib.json"
BIN="${BUILD_DIR}/basalt_vio"

ARGS=(
  --dataset-path "${DATASET}"
  --cam-calib "${CALIB}"
  --dataset-type euroc
  --config-path "${CONFIG}"
  --marg-data "${OUT}/marg_data"
  --show-gui 0
  --save-trajectory euroc
)
if [[ "${NUM_THREADS}" =~ ^[1-9][0-9]*$ ]]; then
  ARGS+=(--num-threads "${NUM_THREADS}")
fi
if [[ "${MAX_FRAMES}" =~ ^[1-9][0-9]*$ ]]; then
  ARGS+=(--max-frames "${MAX_FRAMES}")
fi

printf '%q ' "${BIN}" "${ARGS[@]}" > "${OUT}/command.txt"
printf '\n' >> "${OUT}/command.txt"

cd "${OUT}"
"${BIN}" "${ARGS[@]}" \
  > >(tee "${OUT}/stdout.log") \
  2> >(tee "${OUT}/stderr.log" >&2)

sha256sum "${OUT}/trajectory.csv" "${OUT}/command.txt" > "${OUT}/sha256sums.txt"
echo "Oracle run complete: ${OUT}"
