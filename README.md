# FPGA drone project

Latest rewired bench test: [`vivado/battery lights flight test/`](vivado/battery%20lights%20flight%20test/README.md).
It uses radio JC1, ESC channels JB1-JB4, external lights JB7-JB9, battery
sense JXADC1/JXADC7, SW15 arm, and CH8/SwB prop lights and CH9/SwC head/landing lights. Normal display
shows `CnFG` during IMU setup, then defaults to `rdy` disarmed or `Arnd` armed; SW7 selects RC yaw, SW11 RC roll, SW12 RC pitch, SW8 signed IMU roll, SW13 battery and SW14
error codes (E000 if clear). LD13 is battery status and LD14 error/view status.
CH7/SwA requests show on LD11; NPU output is display-only. CH10/SwD enables
ultrasonic height capture, with OFF-to-manual throttle pickup and a 500ms blend.
LD12 is solid during healthy hold, blinking during pickup/blend/fault, off manual.
Motor channels 0/1/2/3 are back-left/front-left/back-right/front-right.
Lidar C/R on JC7 and D/T on JC8 feeds an 8x8 USB depth viewer; LD9 is lidar
health; LD10 is ultrasonic range status (JC3 trigger/JC4 divided echo).
SW10 shows ultrasonic mm and SW9 the NPU command (Cont/LEFt/rIGt/uP/StoP).
Hover entry requires all four motors selected, actual arming, healthy inputs,
fresh range and throttle >=100/1000. SW15 still immediately disarms. See
[height-hold behavior and failure policy](Drone/FlightController/HOVER.md).
SW11/SW12 now monitor signed RC roll/pitch commands in the hover image;
the X/Y estimator remains excluded and its source is retained. RC axis views
show -500..+500 with neutral zero; they do not apply trim offsets. There is no ESC return status.
Rotor directions and stabilization signs still require bench checks.
SW4 now shows measured yaw speed, SW5 signed yaw correction counts, and SW6
relative heading error. These diagnostic selectors do not affect motor commands.
The current gain trial doubles only the bench heading gain (8 to 16). Reported
yaw hold improved; horizontal motion after stick release is under diagnosis.
Neutral roll/pitch requests level attitude, with no horizontal velocity braking.
See the latest bench guide.
With SW8 ON, SW4/SW5/SW6 instead display measured pitch angle, requested pitch
correction and pitch rate. These views support frame-level and feedback checks;
SW8 alone remains roll and SW12 remains received pitch-stick input.
The earlier test described below retains its original pin assignments.

[`vivado/imu flight test/`](vivado/imu%20flight%20test/README.md) integrates radio on JB1,
IMU CS/MISO/SCLK/MOSI on JA1-JA4, the shared controller's full-control profile,
throttle display, and motor selection. It excludes lidar/NPU. Motor order is
SW0 back-left, SW1 front-left, SW2 back-right, SW3 front-right on JA7-JA10.
Basys3 SW4 controls arming. Its guide covers automatic calibration and mounting /
feedback checks required before flight.

`Drone/` is the source root and `Drone/Drone.v` is the top module. Folders represent
subsystems and contain a matching entry module. Helper modules share that folder;
they do not get individual subfolders. Larger subsystems such as
`PeripheralInterface` contain folders for their component interfaces.

Place shared helpers in the deepest directory common to all their design users.
For example, the radio and lidar share `PeripheralInterface/UartReceiver.v`, while
the lidar and system UART output share `Drone/UartTransmitter.v`. Keep helpers
used by only one subsystem in that subsystem's folder. Move mirrored testbenches
with their source files and update test includes when sharing changes.

```text
Drone/
  Drone.v
  UartTransmitter.v
  FlightController/
    FlightController.v
  NPU/
    NPU.v
  PeripheralInterface/
    PeripheralInterface.v
    IMU/
      IMU.v
      imu_spi.v
      ImuEstimator.v
    MotorControl/
      MotorControl.v
      MotorDriver.v
    UartReceiver.v
    LidarArray/
      LidarArray.v
    RadioReceiver/
      RadioReceiver.v
      IBusDecoder.v
      RCMapper.v
  System/
    System.v
    TickGenerator.v
    UartOutput.v
    SevenSegmentDisplay.v
    BatteryMonitor.v
    LedBlinker.v
```

`System` reads scaled battery voltage through XADC, estimates battery percentage,
and drives a 60-flash/minute beacon. It generates `control_tick` and provides host UART output and the Basys3
seven-segment display driver. `PeripheralInterface` contains the FS-iA6B radio,
SparkFun SEN-15335 ICM-20948 SPI IMU, SEN0628 8x8 lidar array, and four-motor ESC output. `Drone` forwards decoded
controls and the system tick
into `FlightController` and instantiates the whole `NPU` subsystem.
Future NPU components belong under `Drone/NPU/` and are composed by `NPU.v`.

`FlightController` now provides a prototype control path: collective thrust,
forward-only pitch, yaw-rate control, IMU-based roll/pitch PD stabilization,
front-facing NPU action overrides, arming interlocks, and four-motor mixing.
The integrated IMU initializes and polls the sensor, calibrates stationary gyro
bias, and estimates small-angle roll/pitch and body rates. `Drone` uses this
physical sensor path by default. Without calibrated, healthy, fresh IMU samples,
the controller cannot arm. See the [IMU guide](Drone/PeripheralInterface/IMU/README.md)
for SPI wiring, required mounting, startup calibration, parameters, and estimator
limits. The root `imu_spi.v` and `_tb` now live in that subsystem and its mirrored
test folder. See the
[flight-computer guide](Drone/FlightController/README.md) for units, signs, NPU
action codes, fault behavior, parameters, and the prototype's limitations.
`NPU` remains an unavailable placeholder but exposes the future action contract;
`Drone.USE_EXTERNAL_NPU` allows a separate development action source.
`MotorControl` provides four reusable DShot ESC drivers
for the AERO SELFIE 45A 4-in-1 board. `Drone` exposes separate front-right,
front-left, back-right, and back-left ESC signals and their commanded throttles. See the
[motor guide](Drone/PeripheralInterface/MotorControl/README.md) for throttle values, timing, wiring,
startup behavior, and integration.

Defaults target the Basys3's 100 MHz clock, a 1 kHz control tick, and 115200-baud
iBUS. `Drone` forwards all radio parameters, including channel mapping and timeout,
and exposes commands, channel values, ticks, and subsystem status for development.
The top-level `rst` must already be synchronized to `clk`. See the
[radio guide](Drone/PeripheralInterface/RadioReceiver/README.md) for channel ranges,
wiring, protocol behavior, and integration constraints.

The lidar interface configures and polls the sensor over a separate 115200-baud
UART. It publishes complete scans through a double-buffered 64-pixel, 16-bit depth
image. At the top level, set `lidar_read_address` to `row * 8 + column` and read
`lidar_distance_mm`; `lidar_frame_valid` pulses on each new scan. See the
[lidar guide](Drone/PeripheralInterface/LidarArray/README.md) for wiring, timing,
error recovery, and image validity. The image is exposed for future NPU integration.

For board output, send bytes using `uart_data` with the `uart_valid`/`uart_ready`
handshake. Pulse `display_write` to store `display_value` (four hex digits),
`display_digit_enable`, and `display_decimal_points`. Both interfaces are exposed
through `Drone`. See the [system guide](Drone/System/README.md) for message examples,
display writes, USB serial settings, and the partial Basys3 output constraints.

## Tests

Tests mirror paths within the source root, replacing `Drone/` with `tests/` and
adding a `_tb` suffix. There is no extra `Drone` folder under `tests/`:

```text
Drone/Drone.v
tests/Drone_tb.v

Drone/PeripheralInterface/UartReceiver.v
tests/PeripheralInterface/UartReceiver_tb.v
```

Each of the nineteen source modules has a self-checking bench and produces a `.vcd`
beside that bench. Shared simulation-only stimulus lives in `tests/support/` as
`.vh` files. The tests include their own required design sources so the existing
VS Code Verilog runner works from any selected `_tb.v` file.

Run the complete suite with Icarus Verilog and `vvp` on PATH:

```powershell
./Run-Tests.ps1
```

Run one module with `./Run-Tests.ps1 -Module UartReceiver`. The runner checks folder,
file, module, and mirrored-test naming, compiles `Drone` independently of the tests,
then compiles and runs each selected bench. Compilation or assertion failure stops
the run with an error. Compiled `.v.out` files and generated `.vcd` files are ignored
by Git. Relative includes are resolved from each test's own directory, matching
the editor runner.

For Vivado, add all `.v` files under `Drone/` recursively and select `Drone` as top.
The source files do not include one another. Add only the relevant testbench as a
simulation source when using its includes; do not compile those included design
files a second time in the same simulation. The supplied automated tests use Icarus.

## November 18 stabilization test profile

Latest battery/lights test tolerates clipped accelerometer samples by using gyro
attitude prediction for those samples, while retaining critical IMU health and
small-angle-envelope interlocks. Relative yaw hold adds a neutral-stick deadband
and bounded integrated heading error. No compass reference is integrated yet.
The experimental [position estimator](Drone/FlightController/POSITION.md) drives
SW11 X / SW12 Y signed-centimetre diagnostic views only; it is not position hold
and cannot influence motor commands. Motor order/spin were reconfirmed as
BL CW / FL CCW / BR CCW / FR CW (channels 0..3, viewed from above).
