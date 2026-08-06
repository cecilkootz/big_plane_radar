#!/usr/bin/env bash
set -euo pipefail

CORE_VERSION="3.2.0"
SDK_URL="https://dl.espressif.com/AE/esp-arduino-libs/esp32-3.2.0-h.zip"
SDK_SHA256="3d6a0ca36c644b1ad1b3b1c405423c7e2fcd947ae8bd2a733d54289c7d540b07"
SDK_TOOL_VERSION="idf-release_v5.4-2f7dcd86-v1"
ROOT="${PLANE_RADAR_HIGH_PERF_ROOT:-$HOME/.cache/big-plane-radar/arduino-high-perf-3.2.0}"
DATA_DIR="$ROOT/data"
DOWNLOADS_DIR="$ROOT/downloads"
USER_DIR="$ROOT/user"
CONFIG_FILE="$ROOT/arduino-cli.yaml"
ARCHIVE="$DOWNLOADS_DIR/esp32-3.2.0-h.zip"
SDK_DIR="$DATA_DIR/packages/esp32/tools/esp32-arduino-libs/$SDK_TOOL_VERSION"
MARKER="$SDK_DIR/.plane-radar-high-perf"
ARDUINO_CLI_BIN="${ARDUINO_CLI_BIN:-arduino-cli}"

mkdir -p "$DATA_DIR" "$DOWNLOADS_DIR" "$USER_DIR"
cat >"$CONFIG_FILE" <<EOF
directories:
  data: $DATA_DIR
  downloads: $DOWNLOADS_DIR
  user: $USER_DIR
EOF

sha256_file() {
  if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    sha256sum "$1" | awk '{print $1}'
  fi
}

if [[ ! -f "$ARCHIVE" ]] || [[ "$(sha256_file "$ARCHIVE")" != "$SDK_SHA256" ]]; then
  echo "Downloading Espressif high-performance SDK 3.2.0-h..."
  curl --fail --location --output "$ARCHIVE.tmp" "$SDK_URL"
  if [[ "$(sha256_file "$ARCHIVE.tmp")" != "$SDK_SHA256" ]]; then
    echo "High-performance SDK checksum mismatch." >&2
    rm -f "$ARCHIVE.tmp"
    exit 1
  fi
  mv "$ARCHIVE.tmp" "$ARCHIVE"
fi

CORE_DIR="$DATA_DIR/packages/esp32/hardware/esp32/$CORE_VERSION"
if [[ ! -d "$CORE_DIR" ]]; then
  "$ARDUINO_CLI_BIN" --config-file "$CONFIG_FILE" core update-index
  "$ARDUINO_CLI_BIN" --config-file "$CONFIG_FILE" core install "esp32:esp32@$CORE_VERSION"
fi

if [[ ! -f "$MARKER" ]] || [[ "$(cat "$MARKER")" != "$SDK_SHA256" ]]; then
  STAGING="$ROOT/sdk-staging"
  rm -rf "$STAGING"
  mkdir -p "$STAGING"
  unzip -q "$ARCHIVE" -d "$STAGING"
  SOURCE_DIR="$(find "$STAGING" -mindepth 1 -maxdepth 1 -type d -name 'arduino-esp32-libs-all-*' -print -quit)"
  if [[ -z "$SOURCE_DIR" ]]; then
    echo "Unexpected high-performance SDK archive layout." >&2
    exit 1
  fi
  rm -rf "$SDK_DIR"
  mkdir -p "$(dirname "$SDK_DIR")"
  mv "$SOURCE_DIR" "$SDK_DIR"
  printf '%s' "$SDK_SHA256" >"$MARKER"
  rm -rf "$STAGING"
fi

echo "High-performance Arduino environment ready: $ROOT"
echo "Arduino CLI config: $CONFIG_FILE"
