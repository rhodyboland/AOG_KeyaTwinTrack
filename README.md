# AOG KeyaTwinTrack

Custom Keya CAN position-mode steering firmware for a Teensy 4.1 on the
AIO v4.5 Micro, intended for a two-track tractor (primarily Challenger 65C).
AgOpenWeb supplies equivalent steering-angle requests; the firmware maps them
to steering-wheel motor positions rather than controlling a conventional axle.

The custom precision-v2 baseline is imported from the sibling
`AW_KeyaTwinTrack_AIO45Micro` package. See [migration review](docs/migration-review.md)
for provenance, AgOpenWeb control mappings and validation limits, and the
[original package notes](docs/precision-v2-original.md) for operating behaviour
and bench acceptance checks. Older root-level overview/issue documents describe
upstream experiments and can be out of date; use these migration notes first.

## Validation

Run `sh tests/run_tests.sh` (requires g++ with AddressSanitizer and UBSan).
Tests compile the actual safety/packet headers and CAN sender with a fake clock
and bus. They do not emulate the complete Arduino sketch or hardware.

For a Teensy compile, install Arduino CLI, the Teensy core, NativeEthernet and
FlexCAN_T4. The supplied package records core 1.57.3 and FQBN
`teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us`.
Run `sh tools/build_firmware.sh`. The script stages the root source into a sketch
folder named `_AW_KeyaTwinTrack`, as required by Arduino. Set `ARDUINO_CLI` to a
CLI executable path if needed and `BUILD_ROOT` to retain output at a chosen path.
The repository name has changed; the existing sketch and Visual Studio project
names are retained to keep the source import focused.

No fresh Teensy build or tractor/bench validation has been performed during this
migration. The original package's prebuilt HEX/BIN are not rebuilt releases of
this repository. Motor release, physical return to centre and takeover still
need hardware validation as described in the original notes.
