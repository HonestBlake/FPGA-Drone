# SparkFun SEN-15335 ICM-20948 interface

The team member's `imu_spi.v` and bench have moved from the repository root into
this subsystem and its mirrored test folder. `IMU.v` combines that mode-0 SPI byte
engine with a corrected initialization/read sequencer, periodic polling, and
`ImuEstimator.v`. `PeripheralInterface` instantiates `IMU`; `Drone` connects the
resulting estimates and health to `FlightController` by default.

This implements the accelerometer and gyroscope needed by the current flight
controller. The breakout's AK09916 magnetometer, auxiliary I2C master, temperature,
FIFO and DMP are not used. There is no compass heading or nine-axis fusion.

## Wiring and mounting

Use the breakout's SPI header pins; the four-pin Qwiic connector carries I2C.
The following proposed JA assignment is supplied in
[`Basys3-IMU.xdc`](../../../constraints/Basys3-IMU.xdc). Merge it only if JA1..4
are free in the board wrapper. These are pin constraints, not a complete board
design or SPI timing closure result.

| Basys3 connection | Breakout pin | `Drone` port |
| --- | --- | --- |
| JA1 / J1 | CS | `imu_spi_cs_n` |
| JA2 / L2 | SCLK (SCL) | `imu_spi_sclk` |
| JA3 / J2 | SDI (SDA) | `imu_spi_mosi` |
| JA4 / G2 | SDO (AD0) | `imu_spi_miso` |
| JA6 / 3.3 V | VCC | Power |
| JA5 / ground | GND | Common ground |

The SEN-15335 breakout includes level shifting for its 1.8 V sensor IO. Use the
breakout header with 3.3 V power and FPGA IO. See the
[SparkFun hardware guide](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide/hardware-overview)
and [schematic](https://cdn.sparkfun.com/assets/c/7/8/f/8/15335_9DoF_Schematic.pdf).
INT is optional and unused because this implementation polls readiness.

Mount the **sensor axes** X forward, Y left, Z up. Use the axes drawing in the
[TDK datasheet](https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf)
to establish orientation. At rest, raw Z should be approximately +16384 and X/Y
near zero. The estimator maps gyro rates to forward/right/down: X, -Y, -Z.
Roll uses positive acceleration Y and pitch uses positive acceleration X.
Positive roll lowers the right side; positive pitch raises the nose; positive
yaw turns right. A differently mounted board requires an upstream axis transform
or a change to the estimator. Do not infer mounting from connector position.

## Sensor startup and data

At the default 100 MHz FPGA clock, SPI runs at 5 MHz. Startup waits 100 ms,
selects bank 0, resets the sensor, waits another 100 ms, disables the primary I2C
interface, wakes all axes, and programs bank 2. Gyro and accel use +/-250 dps
and +/-2g, approximately 50 Hz low-pass filters, aligned output clocks, and
562.5 Hz register updates. Configuration readbacks and WHO_AM_I=0xEA are checked,
then a further 35 ms wake interval is allowed. The identity, scale factors,
register values and timing come from the TDK datasheet linked above.

The sequencer holds CS for at least 500 ns after the last falling clock edge
and leaves it high for at least one half-period between transactions. Polling
defaults to 500 Hz. Each poll checks identity and bank-0 INT_STATUS_1 readiness
before a 12-byte burst starting at ACCEL_XOUT_H (0x2D). Reads without readiness
never pulse `imu_raw_valid`. Six signed raw outputs update together. A bad
identity or configuration readback latches the driver error until reset.

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
+/-35 degrees. Reset clears the fault and requires calibration again. The flight
controller then requires healthy arm-OFF followed by arm-ON radio packets at
idle before rearming. Repeated reads of stale sensor registers cannot keep the
flight controller armed.

This estimator is for the existing small-angle control prototype. The sin(theta)
approximation and independent roll/pitch integration incur increasing error with
tilt and simultaneous rotation. It provides no heading, altitude, velocity, or
accelerometer multi-position calibration. Sensor saturation limits the usable
rates/accelerations to the selected ranges. Verify sensor orientation, gains,
FPGA synthesis/resource use and 100 MHz timing on the actual board before flight;
the simulations do not model airframe dynamics or validate physical hardware.

## Configuration and verification

`Drone` forwards `IMU_SPI_CLK_DIV`, `IMU_POWERUP_CYCLES`, `IMU_RESET_CYCLES`,
`IMU_WAKE_CYCLES`, `IMU_POLL_HZ`, `IMU_CALIBRATION_SAMPLES`, and `IMU_TIMEOUT_MS`.
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
