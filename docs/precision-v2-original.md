# AW KeyaTwinTrack — AIO v4.5 Micro — precision v2

Experimental firmware for Teensy 4.1 with an AIO v4.5 Micro, one GPS and an
I2C BNO080/BNO085. Flash `AW_KeyaTwinTrack_AIO45Micro_precision_v2.hex` using
Teensy Loader / Teensy Flasher. Use this firmware instead of the earlier builds.
BIN is included for other tooling; its load address is 0x60000000.

## Precision v2 changes

The AOG UDP request is a signed 16-bit value in hundredths of a degree. Previously,
the decoded float was assigned to int16_t steerAngleSetPoint, truncating it towards
zero (not rounding to nearest). The calculated intendedSteerAngle was also integer.
Both are now float, so sub-degree requests survive to degreesToPosition(). Feedback
angle storage and the intermediate positionToDegrees argument are float as well,
removing the corresponding intermediate truncation without changing the feedback
scaling formula or protocol field units. Final CAN position conversion still rounds
to an integer native encoder count. PGN 253 feedback still uses integer hundredths.

At CPD 30, the existing 10000-count/revolution mapping gives about 0.012 equivalent
AOG degrees per encoder count (0.036 physical wheel degrees). This preserves AOG's
0.01-degree command increments as closely as the final encoder quantisation permits.
The existing Ackermann offset is still added: this is NOT a zero-centre mapping fix.
The current/pressure takeover limitations also remain. Standstill interlocks,
AIO wiring, EEPROM layout, CPU clock, CPD scaling and motor timing are retained.

Tests now include positive/negative hundredth-degree decoding, fractional CAN targets
through the actual sender, and fractional feedback. Host sanitizer tests, the Teensy
build, HEX checksums and HEX-versus-BIN verification passed. No hardware test performed.

Before changing tuning, keep settings fixed and compare bench requests of 0.00,
+0.25, +0.50, +0.75 degrees and the negative equivalents. Expect distinct targets;
do not expect physical centre at zero until the separate offset issue is fixed.

## Operator behaviour

| Condition | Result |
|---|---|
| Startup, or first AOG packet already says ON | Motor remains disabled; start with AOG OFF, then engage |
| Speed below 1.0 km/h | Release motor and cancel the engagement |
| Speed 1.0 to below 1.5 km/h | Existing steering may continue; new engagement is rejected |
| Speed 1.5 through 25.0 km/h | A fresh engagement request can be accepted |
| Speed above 25.0 km/h | Cancel engagement and release motor (retains upstream limit) |
| Returning to working speed after a stop/fault | Does not by itself engage steering |
| No valid AOG steering command for 500 ms | Cancel engagement and send disable |
| No valid Keya heartbeat for 1000 ms, or motor fault | Cancel engagement and attempt disable |
| Local maintained switch OFF, button disengage, or configured takeover | Cancel engagement and send disable |

To re-engage, drive at least 1.5 km/h and toggle the AOG steering control OFF
then ON, or make a fresh press of the configured local engage button. A maintained
switch must be switched OFF then ON; leaving it ON cannot restart steering.
Requests made below 1.5 km/h are discarded, rather than queued for later.
Set the AOG software minimum engagement speed to at least 1.5 km/h. Its own
settings still apply independently. Thresholds here are compile-time constants
in SteeringSafety.h, not adjustments to AOG's settings.

A local engage first reports the local switch as engaged to AOG. Motor torque
is only permitted once AOG also sends ON, within 500 ms of the request. Once
engaged, an AOG OFF command removes motor permission immediately. Input mode
must be configured correctly in AOG: Button, Switch or None/tablet only. The
physical input is debounced for 30 ms, sampled at 25 ms intervals. A held input
at boot does not count as a press.

## What changed and why

1. Replaced the bypassed `gpsSpeed < -1` check and iteration watchdog with an
   explicit, host-tested engagement state machine and elapsed-time watchdog.
   A valid OFF-to-ON AOG transition or fresh local input is consumed once. A
   constant ON command cannot undo a stop/fault lockout.
2. Motor permission is recomputed on every autosteer-loop pass, ahead of motor
   sequencing, from speed, command age, local control, WAS range, motor heartbeat,
   motor faults and valid non-zero calibration settings. A pending zero reset
   also cancels engagement. LED/switch feedback uses the resulting state.
3. Motor disable is sent even when the Keya has not been detected, and repeated
   at 100 ms intervals. Disable takes priority over calibration. A failed local
   enqueue is retried; failed speed/position/enable enqueue latches a separate
   transmit fault until a disable has been accepted and engagement has cancelled.
4. Preserved the upstream 100 ms speed-to-position interval using nonblocking
   sequencing. Stop/disengage can cancel the pending enable during that wait.
   Target precedes enable; sequential CAN frames preserve ordering. Re-engagement
   always sends a target even when unchanged. While engaged, commands refresh
   at least every 100 ms and on target changes.
5. Validate UDP header, complete size, declared payload size and checksum before
   using packets. Only a valid 14-byte steering command refreshes its watchdog.
   Keep correct discovery/subnet packet sizes, so scan/network setup still work.
   Signed target decoding avoids a signed left shift. Precision v2 additionally
   preserves fractional target and feedback angles; the steering mapping remains unchanged.
6. Retained the AIO port correction: main GPS Serial7 at 460800 baud, RTK radio
   Serial3 at 115200, second GPS Serial2, Keya on Teensy CAN3 (AIO CAN1), I2C IMU
   Wire pins 18/19. No second GPS is required; the upstream single-GPS/I2C IMU
   detection is retained.
7. Removed repeated CAN-loss message flooding. Startup log identifies precision v2.

## Important limits

This releases the motor; it does not actively command the wheel to centre.
Physical spring return with the Keya installed must be verified. A CAN disable
accepted by the Teensy driver does not prove the motor received or obeyed it.
A broken CAN link or failed controller cannot be made safe by a software message.
The thresholds use speed reported by AOG, not an independent ground-speed
sensor. Fresh but incorrect/frozen speed reports are outside this timeout's
protection. The timing checks are cooperative, not a certified real-time limit.

An AOG OFF-to-ON transition is the tablet's engagement request. The protocol
contains no indication whether a person, an automatic feature or another sender
caused it. This build prevents restart from a continuously held ON signal; it
cannot distinguish a new automatic OFF/ON sequence from a manual tablet toggle.
Avoid automatic re-engagement features if deliberate re-engagement is required.

The upstream steering calibration, encoder scaling, deadband/slack mapping,
GPS-based offset correction and speed-mismatch takeover heuristic have not been
redesigned or validated here. Do not interpret this fix as validation of the
whole experimental twin-track controller. No hardware or tractor tests were
performed. GNSS/IMU freshness, RTK quality and direction remain governed by the
upstream/AOG behaviour; this is not a new independent navigation monitor.

## Bench acceptance checks before field use

Keep the motor disconnected from the tractor steering shaft. Initial bench
movement tests can use AOG's simulator; never inject simulated speed into a
tractor-connected motor for this test. Have motor power independently removable.

1. At zero reported speed, attempt tablet and local engagement: no torque.
2. At 1.4 km/h, attempt engagement, then raise speed to 2 without another request:
   no torque. Toggle OFF/ON above 1.5: steering responds.
3. While engaged, lower speed to 1.2: it continues. Lower to 0.9: it releases.
   Raise speed back to 2 while AOG remains ON: no restart. Make a fresh request:
   it responds, even if target angle has not changed.
4. Disengage locally and through AOG at a non-zero target; verify release.
   Test Button and/or Switch according to the actual installation.
5. Stop AOG command traffic (e.g. disconnect tablet Ethernet) while leaving the
   motor CAN link intact: verify release after the 500 ms timeout. Reconnect:
   it must remain off until a fresh request.
6. On the bench, interrupt and restore motor CAN/heartbeat, then verify no
   automatic restart. Removing CAN does NOT guarantee physical release; check
   the motor's independent communication-loss behaviour separately.
7. Change zero calibration while stopped: disable must precede encoder reset;
   it must not enable the motor. Reset only with the shaft physically centred.
8. With the motor coupled and the tractor immobilised using the appropriate
   procedure, verify manual override, independent power cutoff and actual
   return spring behaviour before any low-speed supervised driving test.

## Build and verification

Upstream: https://github.com/lansalot/AW_KeyaTwinTrack
Commit: 49f30659f47ed439d84477de8c2d49386bd437f6.
AIO pin comparison: AgOpenGPS-Official/Boards commit
60e8d894625c1a0dfd0bfeecd8247dc224cb69a8.
Teensy core 1.57.3, Arduino CLI 1.5.2-rc.1, GCC 5.4.1 bundled by PJRC.
FQBN: teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us.
Runtime CPU clock remains the upstream 450 MHz.
EEPROM identifier and layout remain unchanged; existing stored configuration is
retained if its identifier is 2500. A board on another firmware may initialise
these settings, so check them after flashing.

Native tests compile the exact SteeringSafety.h and AogPacket.h. A second suite
compiles the actual zCANBUS.ino against a fake CAN bus and clock, including
combined state-machine/sender stop and timeout scenarios. Tests run with
AddressSanitizer and UndefinedBehaviourSanitizer; leak detection is disabled
because this execution environment cannot support LeakSanitizer. Tests do not
emulate the complete Teensy, GPIO electrical behaviour, Ethernet driver, CAN
hardware queues, AOG UI or motor firmware. Those require the bench checks above.

Build log, tests, source changes and SHA-256 checksums are included. The build
succeeded without compiler warnings; the CLI reports an unavailable optional
Teensy USB discovery helper, which does not affect compilation or the HEX.
