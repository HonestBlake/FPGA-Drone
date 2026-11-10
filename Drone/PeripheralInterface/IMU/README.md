# SparkFun SEN-15335 ICM-20948 interface

The team member's `imu_spi.v` and bench have moved from the repository root into
this subsystem and its mirrored test folder. `IMU.v` combines that mode-0 SPI byte
engine with a corrected initialization/read sequencer, periodic polling, and
`ImuEstimator.v`. `PeripheralInterface` instantiates `IMU`; `Drone` connects the
resulting estimates and health to `FlightController` by default.

This implements the accelerometer and gyroscope needed by the current flight
controller. The breakout's AK09916 magnetometer, auxiliary I2C master, temperature,
FIFO and DMP are not used. There is no compass heading or nine-axis fusion.

## Team update reviewed November 9, 2026

The newly supplied mode-0 `imu_spi.v` byte engine is merged into the shared
driver, with a divider-sized counter and clock-derived timing parameters.
The existing bank/reset recovery, configuration readbacks, CS timing, identity
checks, fresh-data polling, and estimator are retained. The attachment's simpler
sequencer lacks those checks and uses different default filter/update settings.
The project's +/-250 dps, +/-2g, roughly 50 Hz filters and 562.5 Hz output rate
remain aligned with the estimator.

Neither attached module performs gyro calibration or attitude estimation. The
attached `basys3_imu_top` is a raw-value display example with roughly 100 Hz
requests, no automatic power-on reset initializer, and an unprovided `seven_seg`
dependency. The new flight test uses the project's reset, poller, estimator,
and display instead. The original files are preserved as non-design references
under `vivado flight test/references/`. Automatic startup calibration is
described below. The confirmed wire order is CS/MISO/SCLK/MOSI on JA1-JA4.

## Wiring and mounting

Use the breakout's SPI header pins; the four-pin Qwiic connector carries I2C.
The following proposed JA assignment is supplied in
[`Basys3-IMU.xdc`](../../../constraints/Basys3-IMU.xdc). Merge it only if JA1..4
are free in the board wrapper. These are pin constraints, not a complete board
design or SPI timing closure result.

| Basys3 connection | Breakout pin | `Drone` port |
| --- | --- | --- |
| JA1 / J1 | CS | `imu_spi_cs_n` |
| JA2 / L2 | SDO (AD0), MISO | `imu_spi_miso` |
| JA3 / J2 | SCLK (SCL) | `imu_spi_sclk` |
| JA4 / G2 | SDI (SDA), MOSI | `imu_spi_mosi` |
| JA6 / 3.3 V | VCC | Power |
| JA5 / ground | GND | Common ground |

The SEN-15335 breakout includes level shifting for its 1.8 V sensor IO. Use the
breakout header with 3.3 V power and FPGA IO. See the
[SparkFun hardware guide](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide/hardware-overview)
and [schematic](https://cdn.sparkfun.com/assets/c/7/8/f/8/15335_9DoF_Schematic.pdf).
INT is optional and unused because this implementation polls readiness.

Use 3.3 V breakout power with the Basys3, even when its external board supply
is 5 V. A breakout's 5 V supply rating alone does not mean its external signal
levels are safe for the FPGA. Power off before moving supply or signal wires.

The hardware diagnostic measured **level raw Z near +16384**, contrary to the
earlier reported Z-down mounting assumption. The selected profile is now
X forward, Y left, Z up. Use the axes drawing in the
[TDK datasheet](https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf)
to establish orientation. At rest, raw Z should be approximately +16384 and X/Y
near zero. `IMU.MOUNT_FRD=1` remains available to rotate acceleration and gyro
samples by 180 degrees about X for true FRD mounts whose level raw Z is negative.
Raw telemetry remains in sensor axes. `Drone` and `PeripheralInterface` now
default to `IMU_MOUNT_FRD=0`, matching the standalone IMU and measured positive-Z
gravity sign. Physical X/Y directions and gyro feedback signs still require
verification on the frame before flight. The estimator then maps gyro rates to
forward/right/down: X, -Y, -Z. Roll uses transformed acceleration Y and pitch
uses acceleration X.
Positive roll lowers the right side; positive pitch raises the nose; positive
yaw turns right. A different mounting requires the appropriate axis transform.
Do not infer mounting from connector position.

## Sensor startup and data

At the default 100 MHz FPGA clock, SPI runs at 5 MHz. Startup waits 100 ms,
selects bank 0, checks identity, resets the sensor and waits another 100 ms.
It then wakes the device/selects its clock and waits 35 ms before reasserting
SPI-only mode, enabling axes and programming bank 2. Gyro and accel use +/-250 dps
and +/-2g, approximately 50 Hz low-pass filters, aligned output clocks, and
562.5 Hz register updates. Configuration readbacks and WHO_AM_I=0xEA are checked,
with the wake wait preceding configuration rather than following it. The identity, scale factors,
register values and timing come from the TDK datasheet linked above.

The sequencer holds CS for at least 500 ns after the last falling clock edge
and leaves it high for at least one half-period between transactions. Polling
defaults to 500 Hz. Each poll checks identity and bank-0 INT_STATUS_1 readiness
before a 12-byte burst starting at ACCEL_XOUT_H (0x2D). Reads without readiness
never pulse `imu_raw_valid`. Six signed raw outputs update together. A bad
identity or configuration readback latches the driver error until reset.

The driver checks WHO_AM_I before device reset/configuration, then again after
configuration and during polling. `imu_spi` and `IMU` expose `fault_register`,
`fault_expected`, and `fault_observed`: the first failed read comparison,
preserved until reset. The address excludes the SPI read bit; configuration
addresses 01/14 refer to bank 2, while 00/03/06 refer to bank 0. Address FF
indicates an unexpected sequencer state. `IMU.bus_fault` distinguishes driver
failure from estimator/data faults. The flight-test wrapper displays the
register/received byte on fault and uses a 1 MHz SPI clock for wiring diagnosis.

Hardware diagnosis found `03.00`: USER_CTRL returned 00 after an expected 10,
despite correct WHO_AM_I. This identifies the added interface-configuration
check as the stopping point; it does not establish a sensor silicon fault.
The wake/configuration order now follows a reset -> wake -> settle -> configure
sequence. A failed USER_CTRL check gets one delayed rewrite attempt; persistent
failure still latches the diagnostic and blocks arming. A simulated wake
transition that clears USER_CTRL reproduced 03.00 with the former ordering and
passes with the new ordering. This fixture is a regression scenario, not a
measurement proving the physical chip's internal cause. Tests also inject a
single dropped write and a persistent write failure, checking recovery and
fault gating respectively.

After the wake-order update, physical initialization passed (LD8/LD9 on).
Calibration diagnostics then measured level raw Z near +16384 while the earlier
FRD transform expected negative Z. With IMU_MOUNT_FRD=0 selected throughout the
project, the user confirmed LD10/LD11 on and LD12 off after reset: physical
startup calibration and fresh estimates now pass. This does not validate flight
gains or establish the physical X/Y and gyro feedback directions.

`imu_init_done` and `imu_detected` describe sensor initialization, not estimator
calibration or arming permission. `imu_busy` describes driver activity.
`imu_raw_valid` publishes raw values; `imu_sensor_sample_valid` publishes the
estimated attitude/rate values. Both are one FPGA clock pulse.

## Calibration, estimates, and faults

Keep the board still and level at startup. The estimator requires 256 consecutive
acceptable samples (roughly 512 ms at the default poll rate). The level window
allows X/Y within 1024 counts and Z within 5% of 1g; all raw gyro axes must remain
within 262 counts (about 2 degrees/s). Movement outside this window restarts the
average. This is a stationary gyro-bias calibration and gravity initialization,
not a full accelerometer scale/misalignment calibration. Slow motion within the
window can bias it, so startup must actually be stationary.

After calibration, gyro counts minus measured bias become centidegrees/s using
100/131. Signed 64-bit intermediate arithmetic avoids negative endpoint overflow.
Gyro integration uses elapsed FPGA clocks between samples and retains 16 fractional
bits in the attitude. A complementary correction of 1/256 per sample pulls
roll/pitch toward the small-angle gravity estimate. Correction is applied only
when acceleration magnitude is 0.8..1.2g and sensor Z exceeds 12000 counts.
Linear acceleration within that window can still disturb the estimate.

`imu_sensor_valid` requires a calibrated, fresh estimate and a healthy sensor.
`imu_sensor_fault` latches on bus error, stale samples (100 ms by default),
raw-axis clipping, unrepresentable rates, or estimated roll/pitch outside
+/-35 degrees. Reset clears the fault and requires calibration again. Optional
`RECOVERY_ENABLED=1` with `recover_request=1` also allows a fresh stationary sample
from a ready, non-faulted driver to clear estimator state and start a complete
new calibration. The flight-test wrapper grants permission only while SW4 is
OFF, radio is valid and throttle is below 10. Production defaults remain zero;
its recovery input is tied low. This does not reset or recover a latched SPI
driver fault. The flight
controller then requires healthy inputs and an arm-OFF followed by arm-ON request
at idle before rearming. The Basys3 flight test supplies that request from SW4.
Repeated reads of stale sensor registers cannot keep the
flight controller armed.

This estimator is for the existing small-angle control prototype. The sin(theta)
approximation and independent roll/pitch integration incur increasing error with
tilt and simultaneous rotation. It provides no heading, altitude, velocity, or
accelerometer multi-position calibration. Sensor saturation limits the usable
rates/accelerations to the selected ranges. Verify sensor orientation, gains,
FPGA synthesis/resource use and 100 MHz timing on the actual board before flight;
the simulations do not model airframe dynamics or validate physical hardware.

The new flight-test hardware build found that the existing wide estimator
arithmetic does not close at 100 MHz. Its board wrapper derives a constrained
10 MHz core clock from the 100 MHz oscillator and passes that actual frequency
to all shared modules, keeping the 500 Hz sample / 1 kHz control rates. Do not
assume simulation at a 100 MHz clock establishes estimator timing closure.

## Configuration and verification

`Drone` forwards `IMU_SPI_CLK_DIV`, `IMU_POWERUP_CYCLES`, `IMU_RESET_CYCLES`,
`IMU_WAKE_CYCLES`, `IMU_POLL_HZ`, `IMU_CALIBRATION_SAMPLES`, `IMU_TIMEOUT_MS`, and
`IMU_MOUNT_FRD`.
Defaults derive from `CLK_FREQ`. `FC_IMU_TIMEOUT_MS` independently limits how long
the flight controller accepts its last estimator sample. Simulation can shorten
startup waits and calibration; keep production delays for hardware.

`USE_EXTERNAL_IMU=0` selects the physical IMU by default. The old `imu_*` input
estimate ports remain as a bench interface when `USE_EXTERNAL_IMU=1`; the physical
`imu_sensor_*` diagnostics remain observable. External estimate ports can be tied
low in a hardware wrapper. They are ignored with the default selection. All
numeric estimates and samples are synchronous to `clk`; top-level reset must
already be synchronized.

Run `./Run-Tests.ps1 -Module '*imu*'` for SPI, estimator and combined tests.
Run `./Run-Tests.ps1 -Module Drone` for two complete control paths: the existing
external-estimate tests, and physical SPI sensor bytes plus serial radio packets
through calibration, arming, stabilization and decoded four-channel DShot.
Tests cover wrong-bank reset, configuration readback failure, signed bursts,
bus timing, missing readiness, disconnects, gyro bias/signs/units, gravity
convergence, acceleration rejection, timeouts, reset and explicit rearming.
