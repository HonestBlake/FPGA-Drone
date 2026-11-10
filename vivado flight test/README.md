# IMU-stabilized full-control test

`FlightTest` connects the shared radio, physical ICM-20948 SPI driver,
calibration/attitude estimator, flight controller, ESC drivers, control timer,
and display. There is **no lidar/IR or NPU instance**. It compiles the sources
directly from `../Drone/`, so IMU changes apply to the whole project and this
test together. Earlier hardware-test snapshots are preserved.

This is a small-angle control prototype, not a flight-validated autopilot.
It has no altitude/position/heading hold or integral control. Verify mounting,
motor rotation, correction signs, and gains with propellers removed before
flight. Fewer than four selected motors is for bench diagnosis; actual
stabilization requires all four in flight.

## Wiring and frame

| Function | Basys3 | FPGA pin |
| --- | --- | --- |
| Receiver servo iBUS TX | JB1 | A14 |
| IMU bar CS (/CS) | JA1 | J1 |
| IMU MISO | JA2 | L2 |
| IMU SCLK | JA3 | J2 |
| IMU MOSI | JA4 | G2 |
| Back-left ESC signal / SW0 | JA7 | H1 |
| Front-left ESC signal / SW1 | JA8 | K2 |
| Back-right ESC signal / SW2 | JA9 | H2 |
| Front-right ESC signal / SW3 | JA10 | G3 |
| Arm request / SW4 | Onboard switch | W15 |

Use common signal ground. The project's SparkFun SEN-15335 breakout supports
3.3 V header power/logic; bare ICM-20948 IO requires 1.8 V and level translation.
JA5/JA11 are ground and JA6/JA12 are 3.3 V. ESC main battery power stays separate
from FPGA signal pins. Basys3 external power is 5 V. Pins follow
[Digilent's master XDC](https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc).

Power this IMU breakout from **3.3 V**, including when the Basys3 itself uses
its 5 V external supply. A breakout accepting 5 V supply does not establish
3.3 V compatibility of its external SPI signals; supply-referenced level
shifters can expose the FPGA to the higher voltage. Power off before rewiring.
If LD8/LD9 remain off and LD12 is on after reset, initialization or SPI
communication failed before calibration. Check the IMU supply, common ground,
and CS/MISO/SCLK/MOSI connections before changing calibration or control gains.

The physical calibration diagnostic measured level raw Z near **+16384 (4000
hex)**. This contradicts the earlier reported Z-down mounting assumption that
caused the selected transform to reject calibration. `FlightTest`, `Drone` and
`PeripheralInterface` now default to **`IMU_MOUNT_FRD=0`**, the estimator's native
X-forward/Y-left/Z-up profile. Raw telemetry remains in sensor coordinates.
The observed gravity sign is verified; the physical X/Y axes and gyro correction
directions still require a frame check before relying on stabilization.
`IMU_MOUNT_FRD=1` remains available for a true X-forward/Y-right/Z-down mount
whose level raw Z is near -16384. It rotates both accel and gyro axes by 180
degrees about X. Change the profile only to match a verified mounting.
Confirmed motor rotation is **FR/BL CW and FL/BR CCW viewed from above**.
`YAW_MIX_SIGN=-1` selects that yaw mixer; `Drone.FC_YAW_MIX_SIGN` also defaults
to -1 for this airframe. Motor positions match the user's switch mapping.
Verify actual propeller handedness and correction direction before flight.
Wrong axes or motor signs can produce positive feedback instead of leveling.

## Calibration and arming

The raw SPI driver does not calibrate. **`ImuEstimator` automatically averages
gyro bias and initializes roll/pitch from gravity on startup/reset.** Keep the
mounted IMU still and level for roughly one second: initialization takes at
least about 235 ms, followed by 256 acceptable samples (about 512 ms at 500 Hz).
Movement outside the stationary window restarts averaging. Watch LD10 rather
than relying on a fixed delay. This is not six-position accelerometer or compass
calibration; the magnetometer is unused and yaw is rate control.

1. Remove propellers, set **SW4 OFF**, power the ESC, and press/release the center button.
   Leave the mounted IMU still and level.
2. Wait for LD5 (ESC startup), LD6 (radio), LD10 (calibrated), and LD11 (healthy
   fresh IMU). LD12 must be off.
3. Select any motor combination with SW0-SW3. All four on selects the full mixer.
   Lower throttle below 10; wait for LD13/LD14 (healthy controller / mask accepted).
4. Move **Basys3 SW4 ON with throttle below 10**. LD4 indicates actually armed;
   LD15 shows the SW4 arm request after calibration. Radio CH5/CH6 are unused.
   Start with SW4 OFF, then ON once the health/selection LEDs are ready.
5. Raise throttle and verify correction direction with propellers removed.

Throttle 0-9 always sends stop, even with attitude corrections. Selection
changes, radio loss, sensor faults/staleness, or reset disarm. Lower throttle
below 10 and repeat the SW4 OFF/ON sequence after health recovers. A rejected
high-throttle arm consumes permission; lowering throttle while SW4 stays ON
does not rearm. Center reset restarts sensor calibration and ESC startup.

The fault response is motor stop; it has no controlled-landing routine. Even
one selected motor requires calibration and the flight controller's health checks.

### Recovering an estimator fault without resetting the board

1. Set SW4 OFF and lower throttle below 10 (normally 0000). Keep valid radio
   packets arriving; CH5 remains unused.
2. Put the IMU still and level. A fresh stationary sample with a healthy SPI
   driver clears the estimator fault and restarts its gyro-bias/attitude
   calibration. This resets only estimator state, leaving the receiver, SPI
   configuration and ESC startup state running.
3. Wait for LD10/LD11 on and LD12 off again. Calibration requires another 256
   consecutive accepted samples, roughly 0.5 seconds at 500 Hz. Motion restarts
   that averaging; a persistent invalid sample or missing data cannot recover.
4. Once ready, turn SW4 ON to arm again. Recovery alone never resumes motors.
   If SW4 is turned on before recovery/calibration finishes, cycle it OFF/ON
   again at idle after health returns.

Recovery is enabled only for this wrapper via IMU.RECOVERY_ENABLED=1 and the
disarmed/idle/radio-valid recover_request. The production interface retains
default latched behavior. Persistent SPI identity/configuration faults still
require correcting the connection and a center-button reset to reinitialize
the driver. The estimator cannot clear a driver error that is still asserted.
Sensor ranges, clipping checks, 100 ms timeout and +/-35-degree tilt limits are
unchanged. This feature makes handling faults recoverable; it does not prevent
an armed-flight fault from cutting motor outputs.

## Controls and status

| Channel | Behavior |
| --- | --- |
| CH1 | Signed roll, +/-20-degree target by default |
| CH2 | Forward/nose-down and reverse/nose-up pitch, +/-20 degrees |
| CH3 | Collective 0-1000, with cutoff below 10 |
| CH4 | Signed yaw rate, up to +/-90 degrees/second |
| CH5 | Unused; Basys3 SW4 controls arming |
| CH6 | Unused; no NPU/avoidance |

Roll/pitch deadband is +/-20 mapped counts. `FULL_MANUAL_CONTROL=1` selects this
profile; `Drone` keeps the production policy unless `FC_FULL_MANUAL_CONTROL=1`.
At 1 kHz, each selected motor receives its own mixed DShot300 value. Collective
is `48 + throttle*1999/1000`, with roll/pitch/yaw corrections. This is different
from the previous direct-throttle test, which sent equal values to each motor.

During normal operation the display shows raw mapped throttle in decimal
0000-1000. `----` means no fresh valid radio packet. A number does not
establish IMU health/arming.

While initialization has succeeded but calibration is waiting (LD8/LD9 on,
LD10/LD11/LD12 off), the display instead shows **raw Z acceleration in signed
16-bit hexadecimal**, with the rightmost decimal point lit. For the confirmed
positive-Z profile, level stationary Z should be about `4000.` (+16384).
`C000.` is -16384, the opposite sign. With **LD13 off**, `C001.` is a status code for
**no fresh published sample within 100 ms**; raw data may be missing even though
identity/configuration checks succeeded. With LD13 on, every displayed value
(including C001, which is -16383) is an actual raw Z reading. These calibration
diagnostics take priority over radio dashes.

During this waiting state only, the upper LEDs mean:

| LED | Calibration diagnostic |
| --- | --- |
| LD13 | Fresh raw sample received |
| LD14 | Accelerometer within the estimator's level/gravity window |
| LD15 | All gyro axes within the estimator's stationary window |

LD14 requires raw X/Y within +/-1024 and transformed Z between +15564 and +17204
(raw positive Z for the default profile). LD15 requires each raw gyro axis within +/-262. All conditions
must hold for 256 consecutive samples. After calibration, normal throttle and
the usual LD13-LD15 meanings return. Fault displays still take priority.

With LD12 on, the display instead shows the failed SPI register and received
byte in hexadecimal, separated by a decimal point: **RR.VV**. The first failure
is held until center reset. This diagnostic takes priority over throttle and
radio-loss dashes; F is displayed as a hexadecimal F rather than a dash.

| Fault display | Meaning |
| --- | --- |
| `00.00` | Identity register returned 00; expected EA |
| `00.FF` | Identity register returned FF; expected EA |
| `00.xx` | Other unexpected device identity |
| `03.xx` | USER_CTRL readback mismatch; expected 10 |
| `06.xx` | Power-management readback mismatch; expected 01 |
| `01.xx` | Bank-2 gyro configuration mismatch; expected 19 |
| `14.xx` | Bank-2 accel configuration mismatch; expected 19 |
| `FF.xx` | Unexpected internal sequencer state |
| `E0.01` | Estimator sample timeout (100 ms) |
| `E0.02` | Gyro raw-axis clipping |
| `E0.03` | Accelerometer raw-axis clipping |
| `E0.04` | Estimated roll/pitch exceeded +/-35 degrees |
| `E0.05` | Computed angular rate outside signed 16-bit range |
| `E0.06` | Driver/bus fault received by estimator; SPI comparison display takes priority |
| `E0.07` | Sensor stopped reporting ready after calibration |

All-00/all-FF responses can result from power, wiring, level-shifting or signal
problems; they alone do not prove which component is at fault. The driver now
checks identity before resetting/configuring the sensor as well as afterwards.
The flight-test SPI clock is reduced from 5 MHz to **1 MHz** to increase timing
margin with jumper wires and breakout level shifters. Sensor polling stays
500 Hz. Other shared-driver instances retain their configured/default SPI rate.

The board reported `03.00` after a successful identity read. The shared startup
sequence now waits for wake/clock startup before writing USER_CTRL, then verifies
the same expected value. One delayed rewrite is allowed on a USER_CTRL mismatch;
a persistent mismatch still blocks initialization and is displayed. All other
identity, wake, scale/filter, data freshness and arming checks remain active.
The user also found a missing shared ground during USB-powered testing. After
connecting IMU/Basys3 common ground and resetting, the same 03.00 remained;
the wake-sequence change therefore addresses an unresolved configuration failure.

| LED | Meaning |
| --- | --- |
| LD0-LD3 | Selected BL, FL, BR, FR motors |
| LD4 | Flight controller armed |
| LD5 | ESC startup stop interval complete |
| LD6 | Radio valid |
| LD7 | One blink per second |
| LD8 | IMU detected |
| LD9 | IMU initialized |
| LD10 | Calibration complete |
| LD11 | Fresh calibrated estimate valid |
| LD12 | Latched IMU fault |
| LD13 | Controller command inputs healthy |
| LD14 | Mask accepted at idle |
| LD15 | SW4 arm request (after calibration) |

## Attached code and shared integration

The supplied files are preserved under `references/` as non-design `.txt`
files. The incoming mode-0 byte engine is merged into shared `imu_spi.v` with
a divider-sized counter. Existing device/bank reset, configuration readbacks,
CS hold timing, data-ready gating, periodic identity checks, and faults remain.
The shared +/-250 dps, +/-2g, roughly 50 Hz filters and 562.5 Hz output settings
stay matched to the estimator. See the
[TDK datasheet](https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf).

Neither attachment calibrates or estimates attitude. The supplied top is a
roughly 100 Hz raw-value display demo, lacks an automatic startup reset, and
references an unprovided `seven_seg` module. The project poller, reset, estimator,
and display are used instead. Shared `FlightController` adds opt-in full control
and roll-setpoint telemetry; `Drone` forwards the new parameters. The shared
display adds optional F-as-dash rendering with unchanged default hex behavior.
The shared IMU XDC now follows CS/MISO/SCLK/MOSI on JA1-JA4.

## Build and verification

The board oscillator remains 100 MHz. This wrapper divides it by ten and runs
the shared control path at **10 MHz**. The existing unpipelined estimator's wide
arithmetic failed a 10 ns cycle in the first hardware build; a 100 ns control
cycle accommodates that arithmetic without changing its numerical behavior.
The XDC declares the actual generated clock; there are no multicycle exceptions
on the estimator. Sensor polling remains 500 Hz and control/DShot updates remain
1 kHz, with timing parameters derived from the actual core frequency. SPI is
1 MHz in this diagnostic revision. DShot300 rounds to 33 core cycles/bit, with 12/25 cycles high for
zero/one (1.2/2.5 us). The UART rounds to 87 core cycles/bit at 115200 baud.
Do not change `CORE_CLK_DIV` without updating the generated-clock constraint.
Shared modules always require `CLK_FREQ` to describe the clock actually supplied
to them; the wrapper handles that frequency forwarding.

From the repository root:

```powershell
& '.\vivado flight test\Run-Test.ps1'
& '.\Run-Tests.ps1'
```

From this directory in a Vivado-enabled shell:

```powershell
vivado -mode batch -source build_bitstream.tcl
```

Project: `build/flight_test.xpr`; design top: `FlightTest`; simulation top:
`FlightTest_tb`. The source list is `sources.tcl` and excludes lidar/NPU.
Bitstream path: `build/flight_test.runs/impl_1/FlightTest.bit`.
The build script does not program hardware. To program the connected project's
Basys3 persistently, run `vivado -mode batch -source program_flash.tcl`.
That separate script saves a complete flash backup, writes and verifies the new
image, then checks FPGA startup. In QSPI mode it boots the image from flash.

The integration simulation sends actual UART packets and physical SPI sensor
responses through calibration, full signed controls, position-to-wire mixing,
decoded DShot packets, display, cutoff, motor masks, arming, sensor disconnect /
stale data, radio timeout, reset, and explicit rearming. The full project suite
also checks the default production profile. No airframe dynamics or flight
gain validation is included.

`FlightClock_tb` separately checks the production 100 MHz input, divide-by-ten
clock, physical SPI calibration, real 1 kHz tick/DShot300 timing, and an
independently timed 115200 baud radio packet. `Run-Test.ps1` runs both benches;
the full integration bench uses a clock bypass and shortened startup/calibration
parameters to keep fault/control tests fast.

The board clock is 100 MHz and the generated logic clock is 10 MHz. External
wiring, motor rotation, correction directions and airframe dynamics still
require physical verification.

The previous image was verified on 2026-11-09 with the confirmed FRD mounting and reversed rotor
layout: all 19 shared project benches and both flight-test benches pass.
The routed Basys3 build meets setup (WNS +8.383 ns), hold (WHS +0.045 ns),
and pulse-width constraints, with zero DRC errors and no critical warnings.
The final image was programmed and verified in the connected Basys3's flash;
QSPI boot mode 001 and successful flash boot (DONE=1) were confirmed.
The programming record is `build/program_final.log`. Complete pre-write flash
backups are retained under `build/flash_before_flight_test_*.bin`; the earliest
backup preserves the previous direct-throttle motor test.

The subsequent 1 MHz diagnostic revision passes all 19 shared benches and both
flight-test benches, including actual display decoding of startup ID failures,
disconnects, hexadecimal FF, decimal-point placement and estimator faults.
Its routed build meets setup (WNS +8.188 ns), hold (WHS +0.055 ns), and
pulse-width timing, with zero DRC errors and no critical warnings.
Build record: `build/build_diagnostic.log`.
The first diagnostic upload attempt (`build/program_diagnostic.log`) found no
connected JTAG target and stopped before programming. Once reconnected, the
diagnostic image was written and verified successfully; QSPI boot mode 001 and
flash startup with DONE=1 were confirmed. Record:
`build/program_diagnostic_retry.log`. Pre-write backup:
`build/flash_before_flight_test_20261109_150830.bin`.

The wake/configuration-order revision passes the 19 shared benches and both
flight-test benches. Regression tests cover configuration lost across a wake
transition, one dropped USER_CTRL write recovering through a bounded retry,
and persistent failure still producing 03.00 and blocking initialization.
Its routed timing meets setup (WNS +7.809 ns), hold (WHS +0.043 ns), and
pulse-width requirements, with no critical warnings or DRC errors.
Build record: `build/build_wake_fix.log`.
This revision was written and verified in the connected board's flash and
booted successfully in QSPI mode (DONE=1). Record: `build/program_wake_fix.log`.
Backup: `build/flash_before_flight_test_20261109_151934.bin`.
Successful configuration/calibration on the physical IMU still requires the
post-upload display/LD8-LD12 observation; flash DONE only verifies FPGA startup.

The user confirmed LD8/LD9 on with LD12 off after the startup update, so sensor
initialization now passes. Calibration remained pending. The subsequent
calibration-status revision exposes raw Z/freshness and the two stationary
windows without relaxing them. Both integration benches pass, including
no-ready-data and moving-gyro cases. Routed setup WNS is +8.122 ns, hold WHS
is +0.094 ns, with no DRC errors or critical warnings.
Build record: `build/build_calibration_diagnostic.log`.
Programming and flash verification succeeded, followed by QSPI boot (DONE=1).
Record: `build/program_calibration_diagnostic.log`; pre-write backup:
`build/flash_before_flight_test_20261109_154935.bin`.

The user then measured raw Z near 4000 hex with LD13/LD15 on and LD14 off.
This confirmed fresh samples and acceptable gyro rates, with the old negative-Z
mounting profile rejecting the positive level gravity value. Defaults now use
IMU_MOUNT_FRD=0 throughout the project and wrapper. All 19 shared tests and both
flight-test benches pass; routed WNS is +8.188 ns and WHS +0.054 ns, with no
DRC errors or critical warnings. Build record: `build/build_mount_fix.log`.
Gravity sign is measured; physical X/Y and gyro feedback directions remain to
be verified before flight.
The positive-Z revision was programmed, verified and booted from flash
(QSPI mode 001, DONE=1). Record: `build/program_mount_fix.log`; backup:
`build/flash_before_flight_test_20261109_155857.bin`.
The user confirmed successful physical calibration after center reset:
LD10/LD11 on and LD12 off. This verifies initialization and a fresh calibrated
estimate on the actual IMU. Physical X/Y directions, motor correction signs and
flight gains still require verification before flight.

## Current SW4 arming revision

SW4 is the flight-test arm request; radio CH5/CH6 are unused. SW0-SW3 remain
the inclusive BL/FL/BR/FR selection mask. LD4 indicates actually armed, and
LD15 shows SW4 after calibration (it retains gyro-window status while waiting
for calibration). Keep SW4 OFF while selecting motors at idle, then ON once
ready. Selection changes or health loss require another OFF/ON sequence at idle.
The production Drone interface keeps its separate radio arm mapping.

All 19 shared benches and both flight-test benches pass. The integration test
holds CH5 at an unconfigured 1500, varies it to both endpoints without affecting
SW4 arming, and checks immediate SW4-OFF stop without another radio packet.
Routed setup WNS is +8.303 ns and hold WHS +0.087 ns, with no critical warnings
or DRC errors. Build record: `build/build_sw4_arm.log`.
The SW4 image was written and verified in flash and booted in QSPI mode 001
with DONE=1. Programming record: `build/program_sw4_arm.log`; prior-image backup:
`build/flash_before_flight_test_20261109_163541.bin`.

## Estimator recovery revision

The flight-test wrapper enables disarmed, idle, radio-valid recovery of
estimator faults. Fresh stationary data restarts estimator calibration without
resetting the SPI driver, receiver or ESC startup. Sensor ranges and limits
are unchanged. A deliberate SW4 OFF/ON sequence after health returns is still
required; recovery never resumes thrust by itself. SPI driver faults remain
latched until reinitialization.

All 19 shared benches and both flight-test benches pass. Recovery tests cover
active faults, moving/unready data, denied permission, complete recalibration,
SW4 ON/high-throttle rejection, no automatic motor restart, and explicit
rearming. Distinct timeout, gyro clipping, accel clipping and tilt codes are
tested. Routed WNS is +8.122 ns and WHS +0.094 ns, with no critical warnings or
DRC errors. Build record: `build/build_recovery.log`.
The recovery image was programmed and verified in flash, then booted in QSPI
mode 001 with DONE=1. Record: `build/program_recovery.log`; pre-write backup:
`build/flash_before_flight_test_20261109_175443.bin`.
Physical recovery confirmation is pending; the automated tests cover complete
estimator recovery and the serial-radio/SPI-to-DShot path.
