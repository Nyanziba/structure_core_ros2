#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_BIN="${TMPDIR:-/tmp}/vendor_burst_scan"

if ! command -v pkg-config >/dev/null 2>&1; then
  echo "pkg-config is required to compile vendor_burst_scan." >&2
  exit 1
fi

if ! pkg-config --exists libusb-1.0; then
  echo "libusb-1.0 not found via pkg-config." >&2
  exit 1
fi

"${CXX:-c++}" -std=c++17 -O2 \
  "${ROOT_DIR}/tools/vendor_burst_scan.cpp" \
  $(pkg-config --cflags --libs libusb-1.0) \
  -o "${OUT_BIN}"

echo "built: ${OUT_BIN}"
exec "${OUT_BIN}" "$@"
