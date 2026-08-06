#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIGH_PERF_ROOT="${PLANE_RADAR_HIGH_PERF_ROOT:-$HOME/.cache/big-plane-radar/arduino-high-perf-3.2.0}"

"$PROJECT_DIR/scripts/setup_arduino_highperf.sh"

export ARDUINO_CLI_CONFIG_FILE="$HIGH_PERF_ROOT/arduino-cli.yaml"
export REQUIRE_HIGH_PERF=1
export RGB_BOUNCE_LINES="${RGB_BOUNCE_LINES:-10}"
export BUILD_PATH="${BUILD_PATH:-$PROJECT_DIR/build/arduino-highperf-${RGB_BOUNCE_LINES}}"

exec bash "$PROJECT_DIR/build_arduino_cli.sh"
