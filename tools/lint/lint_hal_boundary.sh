#!/usr/bin/env bash
# HAL boundary lint (the MCU-free rule of the HardFOC HAL architecture).
#
#   1. lib/api/ and lib/managers/ include no MCU or RTOS-kernel header: no ESP-IDF
#      (esp_*, driver/, hal/, soc/, rom/, sdkconfig.h), no FreeRTOS (freertos/,
#      FreeRTOS.h, task.h, semphr.h, queue.h), no mcu/<family>/ interface wraps,
#      no STM32 Cube, no WS2812 RMT driver. The only exception is the platform
#      binding directory lib/managers/bsp_bind/esp32c6/ (and future
#      bsp_bind/<family>/ directories); bsp_bind/host/ is MCU-free like the rest.
#      RTOS access goes through hf-utils-rtos-wrap (RtosMutex, SignalSemaphore,
#      OsUtility), which is allowed.
#
#   2. Example apps (examples/esp32/main/) use the HAL through lib/api/ and
#      lib/managers/ only: no handlers/, core/, mcu/, base/ or bsp_bind/ include.
#      ESP-IDF and FreeRTOS headers are fine in apps (they own their tasks).
#
# Known violations are listed in tools/lint/hal_boundary_allowlist.txt. The list
# may only shrink: an entry that no longer violates fails the lint (delete it),
# and with --base <git-ref> the lint fails if the list has more entries than at
# that ref (CI passes the pull request's base branch).
#
# Usage: tools/lint/lint_hal_boundary.sh [--base <git-ref>]
# Exit status: 0 clean, 1 violation or allowlist problem.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "${HERE}/../.." && pwd)"
cd "${ROOT}"

ALLOWLIST_REL="tools/lint/hal_boundary_allowlist.txt"
BASE_REF=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --base) BASE_REF="${2:?--base needs a git ref}"; shift 2 ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

INC='^[[:space:]]*#[[:space:]]*include[[:space:]]*[<"]'

# Rule 1: MCU / RTOS-kernel headers above the binding.
MCU_PATTERNS=(
  "${INC}esp_"
  "${INC}driver/"
  "${INC}hal/"
  "${INC}soc/"
  "${INC}rom/"
  "${INC}sdkconfig\.h"
  "${INC}freertos/"
  "${INC}(FreeRTOS|task|semphr|queue|timers|event_groups)\.h[\">]"
  "${INC}([^\">]*/)?mcu/(esp32|stm32)[^\">]*"
  "${INC}stm32[[:alnum:]_]*\.h"
  "${INC}[^\">]*hf-ws2812-rmt-driver/"
  "${INC}(ws2812_cpp|ws2812_effects|rmt_wrapper|ws2812_control)\.h"
  "${INC}([^\">]*/)?handlers/ws2812/"
)

# Rule 2: HAL internals below the managers, included from an example app.
APP_PATTERNS=(
  "${INC}(core|handlers|mcu|base|bsp_bind)/"
)

load_allowlist() {
  ALLOW=()
  [[ -f "${ALLOWLIST_REL}" ]] || return 0
  local line
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line%%#*}"
    line="$(echo "${line}" | xargs)"
    [[ -n "${line}" ]] && ALLOW+=("${line}")
  done < "${ALLOWLIST_REL}"
}

is_allowed() {
  local f="$1" a
  for a in "${ALLOW[@]+"${ALLOW[@]}"}"; do
    [[ "${f}" == "${a}" ]] && return 0
  done
  return 1
}

violations=0
problems=0
VIOLATING=$'\n'   # newline-separated list (portable to bash 3.2)

scan() {
  local area="$1" rule="$2"; shift 2
  local -a patterns=("$@")
  local f pat hit
  [[ -d "${area}" ]] || return 0
  while IFS= read -r f; do
    case "${f}" in
      lib/managers/bsp_bind/host/*) ;;            # MCU-free: scanned
      lib/managers/bsp_bind/*/*) continue ;;      # platform bindings: allowed
    esac
    hit=0
    for pat in "${patterns[@]}"; do
      if grep -En "${pat}" "${f}" >/dev/null 2>&1; then
        hit=1
        if ! is_allowed "${f}"; then
          echo "VIOLATION (${rule}): ${f}"
          grep -En "${pat}" "${f}" | sed 's/^/    /'
        fi
      fi
    done
    if [[ ${hit} -eq 1 ]]; then
      VIOLATING+="${f}"$'\n'
      is_allowed "${f}" || violations=$((violations + 1))
    fi
  done < <(find "${area}" -type f \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.ipp' \) | sort)
}

load_allowlist
scan lib/api "MCU header above bsp_bind" "${MCU_PATTERNS[@]}"
scan lib/managers "MCU header above bsp_bind" "${MCU_PATTERNS[@]}"
scan examples/esp32/main "app includes a HAL internal" "${APP_PATTERNS[@]}"

# Stale allowlist entries: the file no longer violates (or no longer exists).
for a in "${ALLOW[@]+"${ALLOW[@]}"}"; do
  if [[ "${VIOLATING}" != *$'\n'"${a}"$'\n'* ]]; then
    echo "STALE ALLOWLIST ENTRY: ${a} no longer violates; delete it from ${ALLOWLIST_REL}"
    problems=$((problems + 1))
  fi
done

# Shrink-only: no more entries than at the base ref.
if [[ -n "${BASE_REF}" ]]; then
  if base_text="$(git show "${BASE_REF}:${ALLOWLIST_REL}" 2>/dev/null)"; then
    base_count="$(printf '%s\n' "${base_text}" | sed 's/#.*//' | xargs -n1 2>/dev/null | grep -c . || true)"
    now_count="${#ALLOW[@]}"
    if (( now_count > base_count )); then
      echo "ALLOWLIST GREW: ${now_count} entries, ${base_count} at ${BASE_REF}. Fix the include instead."
      problems=$((problems + 1))
    else
      echo "allowlist: ${now_count} entries (${base_count} at ${BASE_REF})"
    fi
  else
    echo "allowlist: ${#ALLOW[@]} entries (none at ${BASE_REF}; this change introduces the list)"
  fi
fi

if (( violations > 0 || problems > 0 )); then
  echo
  echo "FAIL: ${violations} boundary violation(s), ${problems} allowlist problem(s)."
  echo "      MCU and RTOS-kernel headers belong in lib/managers/bsp_bind/<platform>/;"
  echo "      managers reach hardware through bsp_bind/VortexBoardBind.h."
  exit 1
fi
echo "OK: no HAL boundary violations (${#ALLOW[@]} allowlisted app file(s))."
