# Migration review — 2026-10-28

## Provenance and scope

Fork baseline: `49f30659f47ed439d84477de8c2d49386bd437f6`.
The supplied precision-v2 package identifies this exact upstream revision.
Comparison of every supplied firmware source file found only three modified
files (`Autosteer.ino`, `_AW_KeyaTwinTrack.ino`, `zCANBUS.ino`) and two additions
(`SteeringSafety.h`, `AogPacket.h`). Other firmware source matches upstream.
The import retains upstream line endings in the two CRLF sketch files; firmware
content otherwise matches the supplied source. Existing flasher and Visual
Studio projects are retained. The host test include path now targets the root,
and test binaries use an isolated temporary directory removed on exit.

The supplied `precision-v2.patch` only describes the precision increment, not
all custom changes; the complete supplied source was used for this migration.
Original release notes are preserved in `precision-v2-original.md`. Supplied
HEX and BIN both passed their SHA256SUMS checks. They remain in the supplied
package rather than being presented as fresh builds of this repository.

## Behaviour reviewed

- GPS Serial7 at 460800, RTK Serial3 at 115200, GPS2 Serial2; Wire on 18/19,
  Keya CAN3 at 250 kbit/s. CPU remains 450 MHz; EEPROM identifier/layout unchanged.
- Single GPS plus I2C BNO08x operation remains supported by upstream detection.
- Engagement requires a fresh request at 1.5–25 km/h; below 1 km/h, stale
  commands (500 ms), lost heartbeat (1000 ms), motor/transmit faults, takeover
  or invalid calibration revoke permission. Continuous ON cannot restart it.
- Disable takes priority over zero reset; speed/position/enable sequencing
  preserves a nonblocking 100 ms wait. Failed enqueue is checked.
- Signed hundredth-degree requests and fractional feedback retain precision
  until the existing CAN/protocol quantisation.

## AgOpenWeb controls

Reviewed the local AgOpenWeb checkout at
`7e016432457dd5e5cf3d36ffb411721a394f4d12`, especially
`Shared/AgOpenWeb.Services/AutoSteer/PgnBuilder.cs`, `AutoSteerService.cs`, and
`Shared/AgOpenWeb.ViewModels/MainViewModel.cs`. No app files were modified.

| App control / packet | Firmware meaning |
|---|---|
| Steering request, PGN 254 | Signed equivalent angle in 0.01 degrees; speed in 0.1 km/h; guidance bit 0 requests ON |
| Counts per degree, PGN 252 byte 9 | Position multiplier: counts = degrees / 360 × 10000 × (CPD / 10); larger CPD increases target travel |
| Ackermann, PGN 252 byte 12 | Repurposed speed command, integer-cast Ackermann × 10000 / 200 with upstream rounding; also sets additive slack magnitude `int16_t(200 / Ackermann)` |
| WAS zero offset, PGN 252 bytes 10–11 | Any change requests reset of the motor's current encoder position; it is not applied as a continuous signed position trim |
| Gain / max PWM / min PWM | Stored, but not used by the active Keya position sender as a conventional PWM PID |
| External enable mode, PGN 251 | None/tablet, maintained switch, or momentary button; physical input is active low and debounced |
| Current trip threshold, PGN 251 | Uses the existing motor speed-mismatch heuristic, not measured motor current |
| PGN 253 steer switch feedback | Bit 1 high means not requested; low means requested (including pending local handshake). The app's field name `SteerSwitchActive` represents the raw high bit |

The app clamps CPD to 1–255, permits Ackermann 0–200, and uses conventional
Ackermann wording. In this firmware zero Ackermann prevents engagement; the
usual app meaning "100 = neutral" does not apply to this motor mapping.
The app's ordinary axle-calibration wizard should not be treated as a validated
twin-track calibration procedure.

## Findings and limits

1. **Confirmed packet incompatibility:** the supplied validator accepts PGN 251
   only at 14 bytes. This app emits 11 bytes (five payload bytes plus framing and
   checksum). The handler only needs fields through byte 8. A follow-up fix
   should accept the app's 11-byte form as well as the existing 14-byte form,
   preserving header, declared-size and checksum checks.
2. **Nonzero and history-dependent centre:** upstream slack sign is selected
   using the *previous* intended angle, then added to the new negated request.
   At Ackermann 50 the magnitude is 4 equivalent degrees, so a zero request
   does not inherently centre the motor. Preserved for a faithful baseline.
3. **Feedback scaling:** heartbeat position is first scaled by 10/CPD and then
   `positionToDegrees` divides by CPD/10 again. This is not the simple inverse
   of the target conversion; precision v2 preserves that formula. Calibration
   and app interpretation require a separate review.
4. **Range handling:** conversion ends in int16_t without explicit saturation;
   large CPD/angle combinations can exceed its range. Nonzero calibration
   guards gate engagement but heartbeat feedback conversion itself can still
   divide by zero if stored CPD is zero. These inherited cases are not covered
   by the supplied nominal-range tests.
5. **Integration coverage:** host tests exercise headers and the real sender,
   not the complete sketch, electrical inputs, app handshake, CAN queues or
   motor response. A successful driver enqueue does not establish motor release.

## Validation performed here

- Both supplied HEX/BIN SHA-256 checks passed.
- Original host suites passed with `-Wall -Wextra -Werror` and ASan/UBSan.
- Source comparison against the fork and local app protocol review completed.
- No Arduino CLI, Teensy core, NativeEthernet or FlexCAN_T4 installation was
  found locally. A fresh target build could not be reproduced; the original
  package's successful build log is historical evidence only.
- No hardware, bench or tractor test performed. Retain the acceptance checks
  in the original notes before treating this experimental firmware as validated.
