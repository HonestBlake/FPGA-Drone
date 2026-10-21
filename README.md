# FPGA drone project

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
```

`System` generates `control_tick` and provides host UART output and the Basys3
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
