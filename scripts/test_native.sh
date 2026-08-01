#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${TMPDIR:-/tmp}/big-plane-radar-label-layout-tests"

c++ \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$PROJECT_DIR/src" \
  "$PROJECT_DIR/tests/test_label_layout.cpp" \
  "$PROJECT_DIR/src/label_layout.cpp" \
  -o "$OUTPUT"

"$OUTPUT"
