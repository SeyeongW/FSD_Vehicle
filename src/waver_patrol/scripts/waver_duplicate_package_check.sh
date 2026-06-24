#!/usr/bin/env bash
set -euo pipefail

WS="${1:-${WAVER_WS:-$HOME/ugv_ws/FSD_Vehicle}}"
cd "$WS"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

if ! colcon list > "$TMP" 2>/tmp/waver_duplicate_package_check.err; then
  cat /tmp/waver_duplicate_package_check.err >&2 || true
  echo "ERROR: colcon list failed in $WS" >&2
  exit 2
fi

DUP_NAMES="$(awk '{print $1}' "$TMP" | sort | uniq -d)"
if [ -n "$DUP_NAMES" ]; then
  echo "ERROR: duplicate ROS package names are visible to colcon in $WS" >&2
  while IFS= read -r name; do
    [ -z "$name" ] && continue
    echo "-- $name" >&2
    awk -v n="$name" '$1 == n {print "   " $2}' "$TMP" >&2
  done <<< "$DUP_NAMES"
  echo "Add COLCON_IGNORE to duplicate archive trees or build from a single canonical workspace." >&2
  exit 1
fi

echo "PASS: no duplicate package names visible to colcon in $WS"
