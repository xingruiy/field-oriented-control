#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/build_flash.sh [options]

Configure, build, and flash the firmware over SWD using the CMake flash target.

Options:
  -p, --preset NAME       CMake preset to use (default: debug)
  -b, --board NAME        Board name under bsp/ (default: preset default)
      --programmer PATH   Path to STM32_Programmer_CLI
  -h, --help              Show this help

Environment:
  PRESET                  Default preset if --preset is not provided
  BOARD                   Default board if --board is not provided
  STM32_PROGRAMMER_CLI    Default programmer path if --programmer is not provided
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

preset="${PRESET:-debug}"
board="${BOARD:-}"
programmer="${STM32_PROGRAMMER_CLI:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--preset)
      [[ $# -ge 2 ]] || { echo "error: $1 requires a value" >&2; exit 2; }
      preset="$2"
      shift 2
      ;;
    -b|--board)
      [[ $# -ge 2 ]] || { echo "error: $1 requires a value" >&2; exit 2; }
      board="$2"
      shift 2
      ;;
    --programmer)
      [[ $# -ge 2 ]] || { echo "error: $1 requires a value" >&2; exit 2; }
      programmer="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

configure_args=(--preset "$preset")
if [[ -n "$board" ]]; then
  configure_args+=("-DBOARD=$board")
fi
if [[ -n "$programmer" ]]; then
  configure_args+=("-DSTM32_PROGRAMMER_CLI=$programmer")
fi

cmake -S "$repo_root" "${configure_args[@]}"
cmake --build --preset "$preset" --target flash
