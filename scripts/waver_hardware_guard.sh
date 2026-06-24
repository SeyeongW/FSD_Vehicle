#!/usr/bin/env bash
set -euo pipefail

ALLOW="${WAVER_ALLOW_HARDWARE:-0}"
NO_HW="${WAVER_NO_HARDWARE:-1}"
BLOCK_SERIAL="${WAVER_BLOCK_SERIAL:-1}"
DISABLE_SOUND="${WAVER_DISABLE_SOUND_OUTPUT:-1}"

if [ "${1:-}" = "--check-env" ]; then
  shift
  echo "WAVER_ALLOW_HARDWARE=${ALLOW}"
  echo "WAVER_NO_HARDWARE=${NO_HW}"
  echo "WAVER_BLOCK_SERIAL=${BLOCK_SERIAL}"
  echo "WAVER_DISABLE_SOUND_OUTPUT=${DISABLE_SOUND}"
fi

COMMAND_TEXT="$*"
FAIL=0

contains() {
  case "$COMMAND_TEXT" in
    *"$1"*) return 0 ;;
    *) return 1 ;;
  esac
}

block() {
  echo "WAVER_HARDWARE_GUARD_BLOCK=$1" >&2
  FAIL=1
}

if [ "$ALLOW" != "1" ]; then
  if contains "start_serial_bridge:=true"; then
    block "start_serial_bridge requires WAVER_ALLOW_HARDWARE=1"
  fi
  if contains "enable_waver_base_driver:=true" && ! contains "mock_transport:=true" && ! contains "serial_port:=mock"; then
    block "enable_waver_base_driver requires mock_transport or WAVER_ALLOW_HARDWARE=1"
  fi
  if contains "/dev/serial/by-id" || contains "/dev/ttyUSB" || contains "/dev/ttyACM" || contains "/dev/ttyTHS"; then
    block "real serial device token is forbidden in hardware-free mode"
  fi
  if contains "enable_sound_output:=true" || contains "enable_sound_deterrent:=true" || contains "sound_backend:=real"; then
    block "real sound output is forbidden in hardware-free mode"
  fi
  if contains "gpio" || contains "GPIO" || contains "pwm" || contains "PWM"; then
    block "GPIO/PWM token is forbidden in hardware-free mode"
  fi
fi

if [ "$NO_HW" = "1" ] && [ "$BLOCK_SERIAL" = "1" ]; then
  for dev in /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS*; do
    [ -e "$dev" ] || continue
    echo "WAVER_HARDWARE_GUARD_SERIAL_PRESENT=$dev"
  done
fi

if [ "$NO_HW" = "1" ] && [ "$DISABLE_SOUND" != "1" ]; then
  block "WAVER_NO_HARDWARE=1 requires WAVER_DISABLE_SOUND_OUTPUT=1"
fi

if [ "$FAIL" -ne 0 ]; then
  exit 12
fi

echo "WAVER_HARDWARE_GUARD=PASS"
