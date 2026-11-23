# Battery, lights, and flight-control bench test

## NPU confidence update - November 23, 2026

Flight-control tuning is paused. SW9 remains display-only NPU output; no
classifier action is connected to motor overrides.

A command now requires a unique winning raw score >=4096, a runner-up lead
>=4096, and a lead >=25% of the winner's magnitude. All classes, including
Continue, face the same conditions. A rejected frame clears any previous
prediction and displays dashes. LD9 remains lidar freshness and can be solid
while SW9 shows an uncertain result. These are initial raw-score heuristics,
not calibrated probabilities. See [NPU guide](../../Drone/NPU/README.md).

The requested new model files have not been located: the two files currently
in `Drone/NPU` match the previous image's hashes, and GitHub main at `ba7733d`
contains the same contents. Confidence changes are being validated against
that existing model while awaiting the updated file location. No new-model
upload has occurred.

Validation so far:
- Six selected NPU module tests passed, covering raw scores and filtered output,
  all five classes, low-score rejection, absolute/relative boundaries, ties,
  negative scores, full signed-range subtraction, health/reset and disabled mode.
- Lidar UART board simulation passed with actual model arithmetic: accepted
  command display, an ambiguous frame clearing an accepted command, dashes
  with healthy lidar, all five labels, unchanged USB pixels/checksum and
  disconnected motor overrides.
- Existing-model confidence bitstream passed timing and DRC checks:
  16,905 placed LUTs (81.27%), 46 DSPs (51.11%), WNS +6.990ns, WHS +0.022ns,
  zero failing endpoints. Build log: `build_npu_confidence.log`.
- Bitstream SHA256:
  `C5AB95B357CEE4769ACA848CE32ADB3ED7BDBA58EA6150E31E39A17390D64136`.
  This image has not been uploaded. The board retains its previous image;
  the pending request needs the actual new model files before programming.

## Pitch diagnostics - November 23, 2026

Yaw flight feedback after the gain trial: the airframe may move about 10deg
off heading during takeoff, then holds heading; forward lean/drift remains.
No new trim or flight gain is applied by this diagnostic update.

Subsequent operator measurement: frame-level pitch displays -00.1deg; raising
the nose makes the angle positive and lowering it negative. This does not
indicate a large static pitch-reference offset. Startup gyro-bias calibration
does not save the startup inclination as level. Subsequent pitch-rate check:
near zero while stationary/held tilted, positive nose-up motion, negative
nose-down motion. At about +10deg nose-up, correction was about -128 counts,
consistent with the current proportional gain (about -125 at exactly +10deg
and zero pitch rate).

Latest flight observation: a persistent forward lean was not apparent in that
flight. Motion continues forward/back/left/right after a commanded lean and
stick release. Neutral roll/pitch commands request zero attitude; the current
image does not command horizontal braking or feed horizontal velocity/position
into motor control. Ultrasonic hover controls vertical collective only. Check
whether measured pitch promptly returns to near zero during the remaining
motion before changing attitude gains. Faster leveling alone does not cancel
existing horizontal velocity. No gain, trim or motor change follows from this
observation alone.

SW8 acts as a pitch-axis modifier when SW4/SW5/SW6 diagnostics are selected.
With SW8 OFF, their existing yaw readings remain available. SW8 alone still
shows roll, and SW12 alone still shows the received pitch stick.

| Switches ON | Reading | Positive sign |
| --- | --- | --- |
| SW8 + SW4 | Measured pitch angle, degrees to 0.1 | Nose up |
| SW8 + SW5 | Requested pitch correction, motor-command counts before clipping | Requests nose-up torque |
| SW8 + SW6 | Measured pitch rate, deg/s; tenths through 99.9 then whole units | Nose rotating up |

Use only one of SW4/SW5/SW6 at a time. Configuration and SW14 error retain
priority. SW4 wins over SW5, which wins over SW6. Negative values show a minus.
Angle/rate work disarmed without radio or motor selection; correction requires
the normal healthy controller path and reads zero while disarmed or stopped.
Unavailable/stale data shows dashes. Display mode does not change motor control.

First measurement: remove propellers, leave SW15 OFF, and level the actual
frame front-to-back (do not assume the landing feet or IMU PCB are level with
it). After stationary calibration, select SW8 + SW4 and record the exact signed
reading, including decimal point. Gently raise the nose: angle should increase
positive; lower it: negative. SW8 + SW6 should show matching rate signs, and
near zero when held still.

Second measurement, still propellers removed: select all motors, keep radio
hover OFF and yaw/pitch sticks centered, arm at idle, then raise throttle above
cutoff. With the nose held slightly down, SW8 + SW4 should be negative and
SW8 + SW5 positive. Nose up should give positive angle and negative correction
after motion stops. Record the frame-level angle plus a held angle/correction
pair, then disarm. Switch only the display selector during each held angle.

Startup averages gyro bias and initializes attitude from gravity; it does not
learn a frame-level pitch offset. The previously reported +5 pitch-stick value
is inside the existing +/-20 deadband, so it requests level pitch. A nonzero
reading with a truly level frame points toward reference/mounting bias; correct
pitch feedback with insufficient correction suggests a different control or
balance issue. Check that the larger battery has not made the airframe nose-heavy.
Roll/pitch remain PD control without integral correction; attitude leveling
alone does not hold horizontal position or brake existing horizontal velocity.

Pitch diagnostic validation:
- FlightController unit tests passed for latched angle/rate, pitch correction
  and damping signs, sample-strobe isolation, idle/disarmed telemetry and the
  existing motor-control/interlock tests.
- New SPI/iBUS board simulation passed: +/-1.74deg stationary reference stays
  visible after gyro calibration, angle/rate signs and decimal formatting,
  front/rear correction signs, the reported +5 RC input inside deadband,
  full pilot pitch, clipping, stopped-state behavior, health loss and priorities.
- Existing yaw diagnostic simulation passed with +/-187 heading contribution;
  manual yaw scaling, heading reset and motor pairs are preserved.
- Existing flight/battery/lights/IMU/arming and RC-axis display board simulations
  passed, including the original roll view, selector priorities, motor selection,
  pilot control, watchdogs and received pitch-stick view.
- Pitch diagnostic build passed timing and DRC checks: 16,581 placed LUTs
  (79.72%), 46 DSPs (51.11%), WNS +7.456ns, WHS +0.061ns, no failing endpoints.
  No flight-control parameters changed from the preceding yaw-gain image.
- Bitstream SHA256:
  `0A55791486754D196147A3FD4914708DC38B7F6AFDE57018C0BA63476BA5B130`.
- Pitch diagnostic image programmed/verified in flash and booted from QSPI,
  DONE=1. Previous 4MB backup:
  `build/flash_before_battery_lights_flight_test_20261123_014004.bin`.
  Programming log: `program_pitch_diagnostics.log`. Physical frame-level and
  pitch-sign measurements remain to be collected from the operator.

## Yaw heading-gain trial - November 23, 2026

Hardware observations: SW4 reads about -0.2..-0.8deg/s while stationary,
positive during clockwise hand rotation and negative counterclockwise.
After a clockwise turn, SW6 reached about -30 degrees and SW5 about -80
command counts. This confirms the measured yaw and requested correction
directions; it does not measure actual motor torque or establish flight tuning.

The current bench profile sets `YAW_HEADING_KP=16` (previously 8), retaining
the +/-30-degree heading-error bound and `YAW_KP=8` yaw-rate feedback. At zero
measured yaw rate, the maximum heading contribution increases from 93 to 187
signed command counts. The combined yaw correction remains limited to +/-400.
For comparison, the earlier reported -300 yaw-stick input requests -54deg/s
and about -168 correction counts at zero measured rate while resetting heading
error. These approximate hover observations suggest insufficient heading
authority, but cannot identify a weak motor or prove that this gain fixes drift.

This change is confined to the bench profile. Pilot yaw scaling, roll/pitch,
motor order, arming, hover, NPU isolation and shared controller defaults remain
as before. It is a gain trial requiring physical validation. Persistent gyro
bias can still cause slow heading drift; no constant yaw trim is applied.

Before uploading/checking, remove propellers and keep SW15 OFF. After warmup,
reset with the drone still and level and let calibration complete. Repeat the
SW6 clockwise-turn test at active throttle, then inspect SW5: near -30.0deg
and zero yaw rate, expect about -187 counts. Opposite rotation should produce
the opposite sign. Disarm if subsequent flight checks develop yaw oscillation.

Gain-trial validation:
- SPI/iBUS board simulation passed for both heading-error directions:
  +/-30deg produces +/-187 correction counts with the expected CW/CCW motor
  pair commands. Manual -300 yaw still produces -168 counts at zero yaw rate;
  deliberate turns reset heading error, and the total correction still clamps
  at +/-400. Disarm, invalid/stale inputs and display isolation checks passed.
- Existing ultrasonic hover board simulation passed, including capture,
  zero-stick hold, OFF/pickup, 500ms release, sonar failure, disarm/RC stop and
  continued NPU isolation.
- Gain-trial build passed setup/hold timing and DRC checks. Placed utilization:
  16,489 LUTs (79.27%), 46 DSPs (51.11%). WNS +7.528ns, WHS +0.022ns,
  zero failing timing endpoints. The synthesis report confirms heading gain
  16 and yaw-rate gain 8.
- Gain-trial bitstream SHA256:
  `9767055FA95120FC34DD24144217A0DA13C01F37F9184F7ED6C5AB5EE8A37BA9`.
- Gain-trial flash programmed and verified; FPGA booted from QSPI (DONE=1).
  Previous 4MB flash backup:
  `build/flash_before_battery_lights_flight_test_20261123_011001.bin`.
  Programming log: `program_yaw_heading_gain.log`.

## Yaw diagnostics - November 23, 2026

The default image adds three read-only views. Control gains, trims, mixer,
arming and hover behavior are unchanged.

| Onboard switch | Reading | Units / sign |
| --- | --- | --- |
| SW4 | Measured IMU yaw rate | Degrees/second; positive clockwise viewed from above, negative counterclockwise. Shows tenths through 99.9, then whole degrees/second with the decimal point off. |
| SW5 | Requested yaw correction | Signed motor-command counts before individual motor clipping; positive requests clockwise body torque, negative counterclockwise. This is not a power percentage. |
| SW6 | Accumulated heading error | Target minus measured yaw, in degrees to 0.1; limited to +/-30.0 by the existing controller. |
| SW7 | Received yaw stick | Signed -500..+500 channel counts before deadband. |

Configuration and SW14 error view retain priority. Then SW4 wins over SW5,
which wins over SW6, followed by the previous views in their existing order.
Select one view at a time. Negative readings show a minus; positive readings
have a blank sign. For example, SW4 `-10.0` means 10 degrees/second CCW,
SW5 `-093` requests CCW correction, and SW6 `-30.0` is the heading-error limit.

First check, with propellers removed and SW15 OFF: select only SW4. After
calibration, leave the drone still, gently rotate it clockwise viewed from
above, then counterclockwise, and stop. Report the stationary reading and
whether the moving readings have the expected signs. SW4 works while disarmed
without radio traffic; stale/unhealthy IMU data shows dashes.

SW5/SW6 require the normal healthy controller command path (including radio
and a motor selection). They read zero while disarmed or at throttle cutoff.
Heading error also resets during an intentional yaw command outside the
existing +/-20 deadband. With neutral yaw and active throttle, persistent
clockwise motion should produce negative heading error and negative correction.
The heading error is relative gyro integration, not a compass heading.

The existing SW11/SW12 received roll/pitch views remain available. These views
do not add motor offsets or enable the position estimator in motor control.

November 23 validation:
- FlightController and SevenSegmentDisplay unit tests passed, including signed
  telemetry, stopped-state behavior and signed three-digit formatting.
- New physical SPI/iBUS board test passed: yaw-rate signs and auto range,
  heading accumulation/cap, correction direction/limit, health loss, selector
  priority, disarmed readout and unchanged motor commands during view changes.
- Existing flight/arming/battery/light, RC-axis and ultrasonic hover board
  regressions passed, including the 500ms handover and RC/disarm stop paths.
- Placed utilization: 16,505 LUTs (79.35%), 46 DSPs (51.11%). Setup WNS
  +7.299ns, hold WHS +0.049ns; no failing timing endpoints or DRC errors.
- Bitstream SHA256:
  `C1A68D1AF8F2CD4C7853A912CA4AE1235377FBA2509985C6A61F25D72EB3266F`.
- Flash programmed and verified; FPGA booted from QSPI with DONE=1. Previous
  4MB backup: `build/flash_before_battery_lights_flight_test_20261123_004454.bin`.
  Programming log: `program_yaw_diagnostics.log`. Physical yaw signs and drift
  behavior still require the operator's measurements.

## RC axis monitor - November 22, 2026

Current default image: SW7 selects received yaw (CH4), SW11 roll (CH1), and
SW12 pitch (CH2), replacing the throttle and two inactive position views.
Each displays the signed pre-deadband command in -500..+500 units: center is
` 000`, positive 25 is ` 025`, negative 25 is `-025`. Values are clamped channel
counts minus 1500, not measured angles or motor outputs. No offset is applied.
Invalid/stale radio data shows dashes, not a misleading zero.

Select one view at a time. Priority is configuration > SW14 error > SW11 RC roll
> SW12 RC pitch > SW10 ultrasonic > SW9 NPU > SW8 IMU roll > SW13 battery
> SW7 RC yaw > default status. LD7 still indicates radio health; LD11/12 retain
AI-request and actual hover-state indications. View switches cannot change motor
commands, arming, hover or NPU permissions. The optional position-only profile
(ENABLE_POSITION_DIAGNOSTICS=1) retains its older SW11/SW12 X/Y diagnostics; the
uploaded hover profile keeps that estimator excluded and uses these RC views.

Record both centered-stick values and the sustained signed inputs needed to
counter drift. Commands within the controller's existing +/-20 deadband still
appear here even though they do not request an attitude/rate change.

Earlier SW7 throttle / SW11-SW12 unused or position descriptions are superseded
for this default image. RC trim/offset corrections are not being enabled yet.

November 22 validation/upload:
- SevenSegmentDisplay signed-token/glyph tests and physical-iBUS RC monitor
  tests passed: all three channel routes, neutral/near-zero/signed endpoints,
  input clamping, selector priority, timeout dashes and armed-flight isolation.
- Existing flight/battery/IMU display, NPU/USB and ultrasonic board regressions
  passed. The displayed sign and digits use the same latched display sweep.
- Final placed utilization: 16,244 LUTs (78.10%) and 46 DSPs (51.11%). Setup
  WNS +7.622ns, hold WHS +0.019ns; no failing endpoints or DRC errors.
- Flash programmed and verified; QSPI boot completed with DONE=1. Previous
  4MB backup: `build/flash_before_battery_lights_flight_test_20261122_213654.bin`.


## Ultrasonic hover and throttle pickup ? November 20, 2026

SwD / CH10 now activates height capture. Entry requires actual arming, all four
motor selectors, fresh 80..2000mm ultrasonic range, attitude within +/-15deg,
healthy control inputs and pilot throttle >=100/1000. SwD must be observed OFF
then ON. It captures current distance and throttle, with no automatic takeoff.
A ground reading of 120 is 120mm from sensor to ground, not a required zero.

The hover image disables the IMU-only X/Y diagnostic estimator to fit alongside
hover and the NPU (`ENABLE_POSITION_DIAGNOSTICS=0`). SW11/SW12 are inactive in this
image; their source and separate diagnostic capability are retained. Attitude
stabilization and yaw control do not depend on that estimator.

SwD OFF continues height hold until pilot throttle reaches the current holding
command, then blends to manual over 500ms. A zero stick does not cancel hover or
bypass the pending-pickup protection. The effective collective feeds both the
flight controller and motor-idle gate. SW15 OFF still immediately stops motors;
RC/IMU/control failure retains existing disarm/rearm behavior.

Unusable/stale ultrasonic data, excessive height-loop tilt, or an implausible
step freezes the last collective and flags a fault, while permitting manual
pickup. This preserves a command; it cannot maintain height without valid
sensing. LD12: healthy hold solid; pickup/blend/degraded/rejected request blinking;
manual off. E031 reports hover failure; E030 and other existing errors keep their
priority. SW10 remains distance mm; SW9 NPU remains display-only. Height gains
are initial values and have not been flight tuned.

See [HOVER.md](../../Drone/FlightController/HOVER.md). The legacy exact-motor
regression explicitly disables hover; the new `HoverBench_tb` tests the enabled
physical UART/SPI/echo-to-ESC path, including zero-stick pickup and disarm/RC loss.
Older descriptions of hover as request-only or disconnected are superseded.

Hover upload validation (November 20):
- Updated HoverController unit tests passed: actual 120mm offset capture,
  entry/window checks, correction/filter signs, zero-stick hold, live pickup,
  smooth signed handover, recapture, freeze on range/tilt failure, and disarm/reset.
- All six board benches passed, including the actual 500ms handover with CH10
  iBUS traffic, physical echo pulses, SPI IMU and all four decoded ESC outputs.
  Existing NPU display/USB, radio lights, battery, manual controls and board
  clock/serial/DShot timing checks remain passing.
- The first combined hover/X/Y diagnostic build exceeded logic capacity and
  was not uploaded. The fitted image disables only that optional estimator.
- Final placed utilization: 16,172 LUTs (77.75%) and 46 DSPs (51.11%). Setup WNS
  +7.299ns, hold WHS +0.049ns; no failing timing endpoints or DRC errors.
- Flash programmed/verified; FPGA QSPI boot confirmed DONE=1. Previous 4MB
  backup: `build/flash_before_battery_lights_flight_test_20261120_151858.bin`.
- Actual ultrasonic behavior under motor noise and flight gains still require
  physical validation; these simulations do not establish flight performance.


## Display-only NPU update — November 20, 2026

SW9 now shows the latest lidar NPU prediction instead of the center-four
average. Confirmed model classes: 0 `Cont`, 1 `LEFt`, 2 `rIGt`, 3 `uP`, 4 `StoP`.
The NPU uses the supplied weights and biases over a private coherent copy of
the same 64-pixel frame streamed over USB. A fresh complete frame is required;
stale/unhealthy lidar clears the prediction and displays dashes. LD9 remains
the lidar scan-health indicator. Existing higher-priority views are unchanged:
configuration, SW14 error, SW11/SW12 position, and SW10 ultrasonic override SW9.

The classifier is display-only. Both NPU motor-override inputs remain tied low
at the flight controller, including when the radio AI-request switch is on.
Flight-drift behavior and existing IMU fault handling are unchanged by this update.
The production `Drone` top keeps NPU disabled by default; this bench enables it.

See [NPU integration](../../Drone/NPU/README.md) for the model contract and tests.
Earlier sections below describe prior images and are superseded where they
refer to SW9 as a center-distance view or NPU as absent.

November 20 validation and upload:
- All 33 shared-module tests and five board-level benches passed across the
  validation runs. NPU comparisons cover all five scores on 20 independent
  reference frames and all five classes. Physical segment masks, class mapping,
  coherent USB data, stale suppression and motor-override isolation are checked.
- The first cache implementation exceeded logic capacity and was not uploaded.
  A single-port RAM cache preserves the same inference arithmetic and inputs.
- Final placement uses 19,836 LUTs (95.37%) and 79 DSPs (87.78%). Setup WNS
  +7.535ns and hold WHS +0.041ns; no failing timing endpoints or DRC errors.
- Flash programmed and verified; FPGA QSPI boot completed with DONE=1.
  Previous 4MB image: `build/flash_before_battery_lights_flight_test_20261120_133745.bin`.
- A subsequent eight-second COM5 read received no USB depth packets. Physical
  lidar power/status and the visible prediction still require user confirmation;
  successful digital inference tests are not a live sensor accuracy check.

This new top, `BatteryLightsFlightTest`, retains the earlier radio/IMU/controller
test and uses shared modules directly from `../../Drone/`. Earlier tests are
preserved. It adds the existing `Drone/System/BatteryMonitor.v`, external light
outputs, and the rewired ESC/radio pins. The lidar reader is instantiated for bench acquisition only; ultrasonic ranging is active for display only. NPU and hover control remain inactive. Motor positions are confirmed below; rotation and stabilization correction directions must still be checked before flight.

## Wiring

| Signal | Header | FPGA pin |
| --- | --- | --- |
| Receiver servo iBUS TX | JC1 | K17 |
| IMU /CS | JA1 | J1 |
| IMU MISO | JA2 | L2 |
| IMU SCLK | JA3 | J2 |
| IMU MOSI | JA4 | G2 |
| ESC channel 0 / SW0 | JB1 | A14 |
| ESC channel 1 / SW1 | JB2 | A16 |
| ESC channel 2 / SW2 | JB3 | B15 |
| ESC channel 3 / SW3 | JB4 | B16 |
| Headlight MOSFET gate / RC CH9 SwC | JB7 | A15 |
| Landing-light MOSFET gate / RC CH9 SwC | JB8 | A17 |
| Prop-light MOSFET gate / RC CH8 SwB | JB9 | C15 |
| Battery divider output, VAUX6P | JXADC1 | J3 |
| Battery analog ground, VAUX6N | JXADC7 | K3 |
| Arm request / SW15 | Onboard | R2 |

IMU power remains 3.3V; the Basys3 external supply remains regulated 5V. All
signal grounds must share a reference. External light outputs are 3.3V active
high, for MOSFET gates through gate resistors, not direct LED power. Reset drives
all light controls low; each physical gate needs its pull-down. SW6/5/4 are
independent enables. Headlights and landing lights are steady; prop lights blink
at approximately 1 Hz while enabled. The user identified the groups as headlights, landing lights, and prop lights. SW4 enables prop lights and does not arm.

HC-SR04 trigger is JC3/N17 and echo is JC4/P18. Echo MUST use the external
5V-to-3.3V divider; it is not a 5V-tolerant FPGA input. Common ground is required.
The XADC battery divider is a different circuit with a 0..1V input.

## Battery reading

The existing shared BatteryMonitor samples VAUX6 through the XADC. Scaling
assumes **15kΩ from battery positive to sense and 1kΩ from sense to common ground**
(1:16). JXADC7 must also be connected to common ground. The user confirmed these resistor values and the grounded negative input. At a full
12.6V pack, sense should measure about 0.788V. Verify with a meter before relying
on the reading; never apply battery voltage directly to JXADC.

This wrapper chooses 0% at 10.5V and 100% at 12.6V, clamped and linearly mapped
between those points. This is a rough usable-voltage indication, **not measured
remaining capacity**. Motor-load sag affects the result. The shared module's
default endpoints are left unchanged. A battery reading does not arm or disarm
motors and provides no automatic low-battery landing or cutoff.

### Current display: configuration, readiness, battery

SW14 ON selects error view, with highest priority: `E000` when no error,
`RR.VV` on an IMU register fault, `E0.01`..`E0.07` on estimator faults,
`E010` for radio loss after configuration, `E011` for ADC not yet valid, or
`E030` for ultrasonic measurement failure. IMU/radio/ADC errors have priority.
Errors are indicated by LD14 even when this view is not selected.

`CnFG` overrides every view during active setup/calibration. A setup fault
ends configuration so the selected error view can expose the failure. After
configuration, display selectors are prioritized as listed below. With no view
selected, `Arnd` means actually armed and `rdy` means currently permitted to arm; it is motor selection, health, idle threshold, and the required healthy arm-OFF
radio frame are all required for ready.
LD15 is the actual armed indicator. Missing data shows dashes. Errors stop motors
through existing interlocks but no longer force the error-code display.

SW13/SW14 are level selections, not edge-triggered toggles. SW6 controls
headlights, SW5 landing lights, and SW4 prop lights. SW11/12 are unused. Battery status does not
control arming or impose a low-battery cutoff.

## Motor identification and arming

Remove propellers. Start with SW15 OFF and throttle at zero. After IMU
calibration, LD8 must be solid; LD7 must be solid for a valid radio link.
Select one of SW0..SW3, wait for `rdy`, then turn SW15 ON
at idle. Solid LD15 indicates actual arming. Raise throttle gently to identify which
motor that header channel drives. Turn SW15 OFF before changing the selection.
Record physical position and rotation for all four channels before using the
mixer for flight. All four selectors may be enabled together, and the confirmed
position-to-channel mapping is:

- SW0/JB1: back-left
- SW1/JB2: front-left
- SW2/JB3: back-right
- SW3/JB4: front-right

### Arm request versus actually armed

LD15 now means actually armed, not merely the SW15 request. If `rdy` is
shown but LD15 keeps blinking, leave the motor selection unchanged, lower throttle
fully, turn SW15 OFF for two seconds, then ON. Changing motor selection while
SW15 is already ON does not automatically arm.

### Status LEDs

| LED | Meaning |
| --- | --- |
| LD0..LD3 | Solid: corresponding motor selected; blinking: deselected |
| LD15 | Solid: armed; blinking: currently can arm; off: arming blocked |
| LD8 | Solid: IMU calibrated/valid without fault; blinking: configuring, calibrating, or unhealthy |
| LD7 | Solid: valid radio link; blinking: disconnected/invalid |
| LD13 | Solid: valid battery above 20%; blinks: 20% or below; off until ADC valid |
| LD14 | Solid: SW14 error view; otherwise blinks on error, off with no error |
| LD6 | Headlights on = solid; off = off |
| LD5 | Landing lights on = solid; off = off |
| LD4 | Follows blinking prop-light output when enabled; off when disabled |
| LD9 | Solid: recent complete lidar image; blinking: setup, error, or stale data |
| LD10 | Solid: valid ultrasonic range; blinking: no reading, timeout, or invalid range |
| LD11 | AI override request from CH7/SwA; AI inactive |
| LD12 | Hover request from CH10/SwD; hover inactive |

Ultrasonic health indicates a plausible echo range, not target identity or guaranteed ground detection.
Blinking is approximately 1 Hz. Reset blanks the indicators. Battery-low
indication does not stop motors; voltage sag may trigger it under load.

The controller retains full signed roll/pitch/yaw commands, throttle <10 stop,
radio/IMU watchdogs, and disarmed-idle estimator recovery. It assumes the same
IMU mounting profile and rotor directions as the previous test; these and the
gains remain subject to physical verification. Only the status indicators listed above are enabled.

## Build and load

From the repository root:

```powershell
& '.\vivado\battery lights flight test\Run-Test.ps1'
vivado -mode batch -source 'vivado/battery lights flight test/build_bitstream.tcl'
vivado -mode batch -source 'vivado/battery lights flight test/program_flash.tcl'
```

The project uses a constrained 5MHz core from the Basys3 100MHz oscillator.
Build checks include setup/hold timing; inspect DRC before programming.
Programming backs up the connected board's 4MB configuration flash, writes and
verifies the new image, and checks startup. Physical LED polarity, rotor direction, stabilization signs,
and ADC calibration still require observation on the actual hardware.

## Validation and upload record - November 13, 2026

- Shared BatteryMonitor test passed all 4096 ADC codes, endpoint rounding,
  clamping, reset, DRP configuration, and non-default parameters.
- New wrapper integration test passed battery endpoints/midpoint, all eight
  external-light masks, reset-off outputs, SW15 arm mapping (SW4 ignored),
  UART/SPI/mixer/DShot behavior, selections, recovery, and fault interlocks.
- Vivado implementation passed: WNS +8.387ns, WHS +0.053ns; zero DRC errors.
- Programmed and verified the connected Basys3 configuration flash, then
  booted the image from QSPI with DONE=1.
- Previous flash saved as
  `build/flash_before_battery_lights_flight_test_20261113_152455.bin` (4MB).
- Physical battery accuracy, rotation,
  and stabilization direction still require the user's bench observations.

### Throttle-display restoration

- Final confirmed motor order: 0 back-left, 1 front-left, 2 back-right,
  3 front-right. Integration simulation passed, including individual front
  channel routing and throttle-display checks.
- Restored-display implementation passed: WNS +8.188ns, WHS +0.034ns;
  zero DRC errors.
- Upload attempt at 15:40 failed because Vivado found no connected hardware
  target. The restored-display image has not been uploaded; the earlier
  battery-display image remains the last verified flash contents.

### Battery/status display update - November 13, 2026

- Integration simulation passed configuration/readiness glyphs, armed battery
  0/50/100%, ADC-unavailable and radio errors, IMU fault priority, and all
  existing motor/control/interlock checks. Shared display and battery tests passed.
- Implementation passed: WNS +8.200ns, WHS +0.094ns, no failing endpoints.
- Configuration flash programmed and verified; QSPI boot confirmed DONE=1.
- Previous 4MB flash backed up to
  `build/flash_before_battery_lights_flight_test_20261113_173347.bin`.
- Battery accuracy still needs comparison with a physical voltmeter.

### Minimal blinking status indicators - November 13, 2026

- Integration simulation passed both blink phases for configuring/faulted and
  healthy IMU, radio loss/health, battery low/full, selected/deselected motors,
  actual armed/disarmed state, and unused LEDs off. Motor/display checks passed.
- Final implementation: WNS +8.303ns, WHS +0.066ns; timing checks passed.
- Final image programmed/verified in flash and booted through QSPI (DONE=1).
- Previous flash backup:
  `build/flash_before_battery_lights_flight_test_20261113_175146.bin`.

### Selectable throttle/battery/error views - November 13, 2026

- Integration simulation passed default throttle, battery percentage, E000,
  IMU/radio/ADC error view, error/view LEDs in both blink phases, reassigned
  external light controls, and existing motor/arming checks.
- Implementation passed WNS +7.809ns, WHS +0.066ns.
- Flash programming/verification and QSPI boot (DONE=1) succeeded.
- Previous 4MB backup:
  `build/flash_before_battery_lights_flight_test_20261113_182933.bin`.

## Lidar image acquisition (no NPU or flight-control connection)

- Sensor C/R (receive) -> JC7/L17, FPGA `lidar_tx`.
- Sensor D/T (transmit) -> JC8/M19, FPGA `lidar_rx`.
- Common ground; sensor powered according to its board specifications. Ensure
  UART signals reaching the FPGA are 3.3V compatible.
- Select UART mode and power-cycle the sensor after changing its mode switch.
  [DFRobot UART guide](https://wiki.dfrobot.com/sen0628/docs/21559).
- Allow about 6 seconds for initial startup/configuration/settling. LD9 is solid
  only after a complete image, with no subsequent protocol error and a frame
  received within 2 seconds. It blinks on startup, stale data, or failed requests.
- Failed transactions retry using the shared LidarArray recovery. Data remains
  isolated from arming, flight stabilization, NPU and hover control.

The 8x8 depth image is streamed over the Basys3 USB serial connection at
115200 baud, 8N1. No additional USB adapter is required. Install pyserial if needed:

```powershell
python -m pip install pyserial
python 'vivado/battery lights flight test/depth_viewer.py' --port COM5
```

Use the actual Basys3 COM port. Run without --port to list ports. The window
shows all 64 raw distances in mm, red near/blue far, in sensor row order.
Values >=4000 are gray (4000 may mean invalid per sensor firmware). Rotation
and mirroring are not inferred. `--headless --frames 5 --csv scans.csv` prints
five frames and records the raw matrix to CSV. A disconnected sensor produces
no USB image packets; the viewer times out after 20 seconds without a frame.

USB packets: ASCII `L8`, frame counter LE32, 64 distances LE16 in row-major
order, sum of preceding 134 bytes modulo 65536 as LE16 (136 bytes total).
A separate snapshot is copied before serialization; frames arriving while that
snapshot is busy are skipped. USB bandwidth does not slow the sensor polling.
The stream checksum protects the FPGA-to-PC packet, not the sensor's original
UART protocol, which has no checksum.

## Range display selections

- SW10: ultrasonic millimetres, e.g. `0610` is approximately two feet.
- SW9: rounded arithmetic mean of lidar addresses 27, 28, 35, 36 (zero-based
  centre rows 3/4 and columns 3/4). A zero or >=4000mm centre pixel makes the
  centre distance unavailable; raw pixels are still included in USB frames.
- Priority: IMU configuration > SW14 error view > SW10 ultrasonic > SW9 lidar
  > SW8 roll > SW13 battery > SW7 throttle > default ready text.
- Missing/stale/invalid ranges show dashes, never a retained old distance.
- HC-SR04 triggers about every 65ms; a failed measurement invalidates its last
  range. LD10 is solid on a valid range, blinking otherwise. E030 reports a
  measurement failure in error view; a subsequent good measurement clears it.
- Both range sensors are diagnostic only. HoverController remains unconnected;
  neither sensor influences motor control, arming, or stabilization.

## Final temporary switch map

| Switch | Function |
| --- | --- |
| SW0/1/2/3 | Select back-left/front-left/back-right/front-right motors |
| SW4 | Enable blinking prop lights (JB9), LD4 follows output |
| SW5 | Landing lights (JB8), LD5 follows output |
| SW6 | Headlights (JB7), LD6 follows output |
| SW7 | Throttle view (0..1000); LD7 still reports radio health |
| SW8 | Left/right IMU roll view, signed rounded whole degrees; LD8 IMU health |
| SW9 | Centre-four lidar average, mm; LD9 lidar frame health |
| SW10 | Ultrasonic distance, mm; LD10 valid-range status |
| SW11/12 | Unused |
| SW13 | Battery percentage; LD13 normal/low status |
| SW14 | Error view; LD14 selected/error status |
| SW15 | Arm request; LD15 actual armed/disarmed status |

For roll, +20 degrees appears ` 020`, negative as `-020`; the estimator uses
positive right-roll convention. Verify physical direction before relying on it.
Display priority is configuration > SW14 > SW10 > SW9 > SW8 > SW13 > SW7 >
default `Arnd` if armed / `rdy` if disarmed. Default is dashes when neither armed nor currently permitted to arm.

## Final sensor/indicator upload - November 13, 2026

- All four test benches passed: existing flight/radio/motor interlocks plus
  light masks/blink phases, configuration overriding every selected view,
  Arnd/rdy/default and signed roll glyphs, sonar trigger/echo and 612mm display,
  coherent lidar USB snapshots, centre-four averaging and invalid pixels,
  frame checksum/order, timeout/freshness and range-view selection.
- Shared LidarArray, SevenSegmentDisplay and FlightController tests passed;
  can_arm assertions checked healthy OFF-frame permission and armed state.
- Final implementation: WNS +8.387ns, WHS +0.072ns. Timing checks passed.
- Flash programmed/verified; QSPI boot confirmed DONE=1.
- Previous 4MB flash backup:
  `build/flash_before_battery_lights_flight_test_20261113_192146.bin`.
- Physical COM5 capture timed out after 20 seconds without a depth frame.
  Lidar hardware operation is not yet verified; check UART-mode switch,
  sensor power/common ground and JC7/JC8. No image data was captured.
- User observed E030 after upload. This is ultrasonic measurement failure,
  not an IMU or motor-arm inhibit; hardware diagnosis remains in progress.

### Ultrasonic/receiver pin migration - November 13, 2026

After an ultrasonic E030 and the report of an earlier direct 5V echo connection,
receiver TX moved to JC1/K17, ultrasonic trigger to JC3/N17, and divided echo to
JC4/P18. JC2 is unused by this test. This tests an alternative echo pin; prior
JC2 damage is possible but has not been established. Echo still requires the
external divider (2.2k upper from ECHO, 3.3k lower to common ground).

Pin-migration build passed setup/hold timing (WNS +7.809ns, WHS +0.024ns).
Flash programming/verification succeeded and QSPI boot was confirmed DONE=1.
Previous flash backup:
`build/flash_before_battery_lights_flight_test_20261113_220433.bin`.
Functional logic is unchanged from the tested sensor-view version; physical
ultrasonic operation on JC3/JC4 still requires the user's range check.

## Scope trigger mode and ADC zero-reading investigation

This test now selects TRIGGER_US=20 and TRIGGER_ON_STUCK_ECHO=1 on the shared
ultrasonic module. JC3 produces a 20us pulse every 65ms, even when echo is stuck
high; reset still holds it low. No radio, IMU health or arm permission gates
these scope pulses. The default reusable module retains its conservative mode
that avoids triggering into an already-high echo. Normal ranging stays active.

Battery investigation: live hardware XADC configuration confirmed channel 22
(VAUX6), single-channel continuous unipolar mode, acquisition extension, divider
2 at 10MHz DCLK. Pin pair is JXADC1/J3 positive and JXADC7/K3 negative.
The user reported JXADC7 may not have been grounded. JXADC7 MUST connect directly
to common ground: measuring JXADC1 to another ground is insufficient if the
ADC negative input floats. No incorrect scaling/configuration was established.
At 0.670V differential, ADC code ~2744 represents ~10719mV and 10% on this test's
10500..12600mV scale. The <=20% low-battery flag should still blink at that voltage.
Both shared exhaustive ADC tests and the wrapper's explicit 0.67V/10% check
are included in validation. A physical reading after grounding pin 7 remains
necessary before claiming this hardware problem resolved.

### Stable one-decimal battery view

SW13 now displays 000.0..100.0 percent with real tenth-percent calculation.
The ADC uses 256-conversion hardware averaging, followed by 64-reading block
averaging and 4Hz publication. Percentage and low-battery status use the same
filtered published measurement. At 11.36V (~0.710V at 1:16), the configured
10500..12600mV voltage scale predicts about 41.0%; this is not coulomb counting.
The reported near-zero JTAG read occurred with USB power and no battery input,
so it was not evidence of an ADC fault. Physical verification must use a powered
battery divider and JXADC7 tied to common ground.

Filtered decimal battery / continuous trigger upload validation:
- Shared Ultrasonic regression includes repeated 20us/65ms scope pulses while
  echo is stuck high. Shared battery regression includes all 4096 ADC codes,
  integer/tenth percentages, noise averaging, publication cadence and hold.
- All four test-wrapper benches and shared System/Drone integration passed.
- Implementation passed WNS +8.303ns, WHS +0.089ns.
- Flash programmed/verified and QSPI boot confirmed DONE=1.
- Previous 4MB flash backup:
  `build/flash_before_battery_lights_flight_test_20261113_230945.bin`.
- Physical battery percentage must still be checked on external battery power;
  user reported 11.36V pack / 0.71V sense after grounding JXADC7. The expected
  voltage-based percentage is approximately 41.0%, not 88%. No calibration
  offset was invented to fit the unreadable rapidly changing previous display.

## Current radio switch controls - November 15, 2026

This section supersedes the earlier temporary onboard light controls. Set the
transmitter to ten channels with CH7=SwA, CH8=SwB, CH9=SwC, CH10=SwD.

| Switch | Function in this image |
| --- | --- |
| SwA / CH7 | AI request indicator LD11 only; AI cannot influence motors |
| SwB / CH8 | Enable blinking prop lights on JB9, LD4 follows output |
| SwC / CH9 | OFF / headlights (JB7) / headlights + landing (JB8); LD6/LD5 follow |
| SwD / CH10 | Hover request indicator LD12 only; hover cannot influence motors |

Basys3 SW4/SW5/SW6 are now unused. SW15 still arms, SW0..3 still select motors,
and all display selections remain unchanged. Inputs are validated in 900..2100;
two-position ON uses >=1750 and SwC thresholds are 1250/1750. Reset or serial
radio timeout clears lights and mode requests. Verify receiver RF failsafe,
which can continue emitting valid held channel values despite transmitter loss.

Radio switch upload validation, November 15, 2026:
- All four wrapper benches passed. Real UART tests covered every combination
  of SwC's three states and SwA/B/D, checksummed/corrupt packets, invalid aux
  values, timeout clearing, old board-switch isolation, unchanged motor commands
  with AI/hover requests on, and existing flight/arming/watchdog checks.
- Implemented image passed setup/hold timing: WNS +8.122ns, WHS +0.043ns.
- Flash programmed/verified, QSPI boot confirmed DONE=1.
- Previous 4MB backup: `build/flash_before_battery_lights_flight_test_20261115_203627.bin`.
- Physical transmitter switch polarity and actual endpoints still need observation.

## Impact/yaw and position diagnostics - November 18, 2026

The bench IMU now uses ALLOW_ACCEL_CLIPPING=1: a saturated accelerometer sample
is flagged and excluded from gravity correction while gyro attitude integration
continues. It no longer latches E0.03 or requires rearming for that event alone.
Shared IMU/estimator defaults retain their strict clipping behavior for older
profiles. Gyro clipping (E0.02), missing data (E0.01), bus/ready failures, rate
numeric overflow, and the +/-35 degree small-angle estimate envelope still fault
and disarm. These are not sensor-built-in disarm rules; they prevent using an
unavailable or unsupported estimate. The sensor remains +/-2g and +/-250dps.

Relative gyro yaw hold is enabled with a neutral stick deadband and bounded
heading-error feedback. Pilot yaw commands turn normally and recapture the hold
target on release. It resets at throttle cutoff/disarm. It has no compass input
and is not an absolute heading guarantee. Motor order/spin were reconfirmed:
0 BL CW, 1 FL CCW, 2 BR CCW, 3 FR CW viewed from above.

SW11/SW12 now select local X/Y position diagnostic centimetres, three digits
plus sign; origin resets at idle. They follow configuration/error view priority
and precede other selected views. LD11/12 still show AI/hover requests, not
position quality. Unavailable or degraded estimates show dashes. This estimator
requires 64 stationary idle samples after IMU calibration and limits tracking to
10s and +/-15deg diagnostic tilt; it does not influence motor control, arming, AI or hover. See
[POSITION.md](../../Drone/FlightController/POSITION.md) for conventions and limits.

Forward drift has not been diagnosed to a specific calibration, mounting, trim,
CG or propulsion cause. Leveling control does not guarantee stationary position.
Verify level-frame IMU alignment, neutral pilot inputs, motor/prop directions,
and corrective response with props removed before flight testing. No new
position-hold control loop is being activated.

The November 18 diagnostic design required lowering the logic clock to 5MHz
(100MHz/20). XDC and the build guard both require a 200ns generated clock.
Control ticks, sensor polling, watchdogs and UART/SPI/DShot parameters derive
from the new clock; nominal physical rates remain unchanged, subject to integer
baud/pulse rounding. BoardClock_tb verifies the real divider, sonar pulse width,
DShot pulse timing and independently timed 115200-baud reception.

November 18 final implementation / upload validation:
- New PositionEstimator tests cover 3D and XY-only equivalence, bias/gravity
  removal, constant acceleration, yaw rotation, bounded time, missing/clipped
  samples, health gating and origin reset.
- FlightController regression covers relative yaw hold at zero rate, pilot
  turn release/recapture, centred-stick deadband, bounded integral, idle reset,
  mixer sign and existing interlocks. ImuEstimator regression covers strict
  defaults and impact-tolerant profile. Five wrapper benches passed including
  physical SPI impact tolerance without rearming and real 5MHz timing/UART/DShot.
- First general rotation implementation exceeded LUT capacity. Compact model
  at 10MHz missed setup timing; the first 5MHz 3D image exceeded LUT capacity.
  No failing image was uploaded. Final board profile tracks XY only, uses
  sized calibration arithmetic and a constrained 5MHz logic clock.
- Final build passed WNS +7.267ns, WHS +0.038ns, no failing endpoints.
- Flash programmed/verified; QSPI startup confirmed DONE=1.
- Previous 4MB flash backup:
  `build/flash_before_battery_lights_flight_test_20261118_185806.bin`.
- Motor positions/spin were reconfirmed by the user. Physical corrective signs,
  neutral transmitter trim, level sensor alignment and gains still need bench
  checks before flight. Forward drift is not claimed fixed by position diagnostics.
