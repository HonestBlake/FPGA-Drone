# Battery, lights, and flight-control bench test

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
