#!/usr/bin/env bash
set -euo pipefail
echo "Optional: create a stable udev symlink for Waver serial after identifying the USB/Jetson device."
echo "Example rule:"
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="waver_esp32", MODE="0666", GROUP="dialout"'
