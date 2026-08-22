#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LABEL_OUTPUT="${TMPDIR:-/tmp}/big-plane-radar-label-layout-tests"
SCROLL_OUTPUT="${TMPDIR:-/tmp}/big-plane-radar-aircraft-list-scroll-tests"
ROUTE_OUTPUT="${TMPDIR:-/tmp}/big-plane-radar-route-plausibility-tests"
ROUTE_JSON_OUTPUT="${TMPDIR:-/tmp}/big-plane-radar-route-json-tests"
BATTERY_OUTPUT="${TMPDIR:-/tmp}/big-plane-radar-battery-gauge-tests"
AIRPORT_LOOKUP_OUTPUT="${TMPDIR:-/tmp}/big-plane-radar-airport-lookup-tests"

c++ \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$PROJECT_DIR/src" \
  "$PROJECT_DIR/tests/test_label_layout.cpp" \
  "$PROJECT_DIR/src/label_layout.cpp" \
  -o "$LABEL_OUTPUT"

"$LABEL_OUTPUT"

c++ \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$PROJECT_DIR/src" \
  "$PROJECT_DIR/tests/test_aircraft_list_scroll.cpp" \
  -o "$SCROLL_OUTPUT"

"$SCROLL_OUTPUT"

c++ \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$PROJECT_DIR/src" \
  "$PROJECT_DIR/tests/test_route_plausibility.cpp" \
  -o "$ROUTE_OUTPUT"

"$ROUTE_OUTPUT"

c++ \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$PROJECT_DIR/src" \
  -I"$PROJECT_DIR/lib/ArduinoJson/src" \
  "$PROJECT_DIR/tests/test_route_json.cpp" \
  -o "$ROUTE_JSON_OUTPUT"

"$ROUTE_JSON_OUTPUT"

c++ \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$PROJECT_DIR/src" \
  "$PROJECT_DIR/tests/test_battery_gauge.cpp" \
  -o "$BATTERY_OUTPUT"

"$BATTERY_OUTPUT"

c++ \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$PROJECT_DIR/src" \
  "$PROJECT_DIR/tests/test_airport_lookup.cpp" \
  -o "$AIRPORT_LOOKUP_OUTPUT"

"$AIRPORT_LOOKUP_OUTPUT"
