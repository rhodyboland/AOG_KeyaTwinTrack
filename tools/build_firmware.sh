#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_root=${BUILD_ROOT:-$(mktemp -d "${TMPDIR:-/tmp}/keya-build.XXXXXX")}
mkdir -p "$build_root/_AW_KeyaTwinTrack" "$build_root/output"
# Only firmware source belongs in the Arduino staging directory.
cp ./*.ino ./*.h ./*.cpp ./*.c "$build_root/_AW_KeyaTwinTrack/"
printf 'Sketch and build output: %s\n' "$build_root"
"${ARDUINO_CLI:-arduino-cli}" compile \
    --fqbn teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us \
    --output-dir "$build_root/output" "$build_root/_AW_KeyaTwinTrack"
