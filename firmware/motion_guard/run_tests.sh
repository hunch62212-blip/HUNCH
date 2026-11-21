#!/usr/bin/env bash
# Build and run the host test for motion_guard.h (no hardware needed).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
clang++ -std=c++17 -O2 -Wall -Wextra -Werror -I"$here" "$here/test/test_motion_guard.cpp" -o "$tmp/test_motion_guard"
"$tmp/test_motion_guard"
