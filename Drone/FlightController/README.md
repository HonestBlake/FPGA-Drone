# Flight-computer prototype

`FlightController` consumes the mapped radio commands, a fused IMU estimate,
and an optional front-facing obstacle action. It generates four ordinary DShot
throttles for the existing `MotorControl`. It updates stabilization and all four
commands together on `control_tick` (normally 1 kHz). Everything uses `clk`;
`rst` is synchronous and active high.

This is a simulation-tested control prototype. Gains, motor rotation, sensor
orientation, synthesis timing, and behavior on the actual airframe still need
verification. The small-angle PD controller has no integral term, heading hold,
velocity control, altitude hold, or autonomous landing. It is intended to establish
the interfaces and control path before those capabilities are developed.

## Pilot controls

| Radio input | Controller behavior |
| --- | --- |
| `throttle`, 0..1000 | Collective thrust; zero commands all motors to stop, including while armed |
| `pitch_command`, -500..500 | Values above the default deadband of +20 request forward/nose-down pitch, up to 20 degrees; zero and negative values request level pitch |
| `yaw_command`, -500..500 | Yaw rate, up to +/-90 degrees/second; positive turns right |
| `roll_command` | Ignored for pilot control; the IMU feedback still levels roll |
| `arm_request` | Explicit arming switch, subject to the interlocks below |
| `avoidance_enable` | Radio mode switch permits the NPU to override forward motion |
| `frame_valid` | One-clock pulse for a new, complete radio packet; required for arm-switch transitions |
| `rc_valid` | Persistent radio freshness from the existing receiver |

The mapper still exposes roll and signed pitch for radio diagnostics. Verify the
transmitter mapping so pushing the pitch stick forward produces a positive
`pitch_command`. The controller converts that to a negative physical pitch angle.
Turns use yaw so the sensor follows the commanded direction rather than commanding
sideways roll. Leveling does not guarantee that the drone's velocity follows its
heading: wind, inertia, and sideslip remain outside this prototype's feedback.

## Integrated IMU contract

`PeripheralInterface/IMU` now contains the team member's SPI byte engine,
the ICM-20948 initialization/read sequencer, and a six-axis small-angle
complementary estimator. `Drone` routes the physical IMU to this controller by
default. The estimator supplies the following contract after stationary gyro-bias
calibration. See the [IMU guide](../PeripheralInterface/IMU/README.md) for wiring,
mounting and limits. Magnetometer/heading estimation is not implemented.

| Input | Meaning |
| --- | --- |
| `imu_sample_valid` | One-clock pulse publishing an atomic set of all five numeric values below |
| `imu_valid` | Persistent flag: the current attitude/rate estimate is usable |
| `imu_calibrated` | Persistent flag: required sensor calibration and estimator initialization have completed |
| `imu_fault` | Persistent flag: sensor, bus, or estimator error; overrides the other health flags |
| `imu_roll_cd`, `imu_pitch_cd` | Signed 16-bit fused roll/pitch, in centidegrees (100 = 1 degree) |
| `imu_roll_rate_cdps`, `imu_pitch_rate_cdps`, `imu_yaw_rate_cdps` | Signed 16-bit gyro rates in centidegrees/second (100 = 1 degree/second) |

Use a forward/right/down body frame: positive roll lowers the right side,
positive pitch raises the nose, and positive yaw turns right, as viewed from above.
Align the sensor and transform its axes/signs into that frame. A positive pilot
forward command intentionally produces negative `pitch_setpoint_cd`.
Clamp/scale upstream values to the signed 16-bit range; rates are limited to about
327 degrees/second by this representation. The physical driver selects +/-250
degrees/second and reports clipping as a fault. Sampling and filtering must be
validated for the eventual airframe.

The controller latches numeric values only on a healthy `imu_sample_valid` pulse.
Hold the bus stable around that clock edge. Samples arriving on the same edge as
a control tick are used at the next tick. After a fault, a new healthy sample is
required. With no usable sample for `IMU_TIMEOUT_MS` (default 100 ms), the controller
disarms. Merely holding `imu_valid` high cannot keep old samples fresh.

`Drone` retains these estimate input ports for bench use with
`USE_EXTERNAL_IMU=1`. The default zero selects the integrated SPI sensor and
estimator; external flags cannot bypass physical sensor calibration or faults.
`imu_sensor_*` outputs expose the physical estimates and health independently of
the selected source. The integrated path uses `clk` throughout. External
estimates must also be synchronous to `clk`; these ports do not implement a
clock-domain crossing.

## NPU actions and front-facing gating

The NPU publishes `npu_available` as a persistent status, and pulses
`npu_action_valid` with a three-bit `npu_action` on every inference result,
including repeated results. The controller latches the action between pulses.
The enumeration is a single code, not a collection of one-hot flags:

| Code | Action | Effect when overriding |
| --- | --- | --- |
| 0 | CONTINUE | Pass the pilot's thrust, forward pitch, and yaw commands |
| 1 | STOP | Request level pitch and zero yaw rate; preserve pilot thrust and roll/pitch stabilization |
| 2 | DIVERT_UP | Request level pitch and zero yaw rate; add 100/1000 thrust, capped at 1000 |
| 3 | DIVERT_LEFT | Request level pitch and a left yaw rate of 45 degrees/second; preserve thrust |
| 4 | DIVERT_RIGHT | Request level pitch and a right yaw rate of 45 degrees/second; preserve thrust |
| 5..7 | Invalid | Treat as STOP |

UP does not add thrust when pilot thrust is zero. STOP does not disarm.
LEFT/RIGHT replace pilot yaw and cancel forward pitch for the duration of the
action; they are turns, not lateral translations. A fresh CONTINUE result allows
forward pitch again. There is no timed maneuver or stored mission.

An action is applied only when the **original pilot** pitch request exceeds the
forward deadband, avoidance is enabled, and the NPU reports available. Gating
uses the original request so STOP does not cancel its own override on the next
tick. With level/reverse pitch, or avoidance disabled, pilot thrust/yaw continue
unchanged. When the NPU is unavailable, manual control works without avoidance.

If an available NPU has no result yet, or the last result is older than
`NPU_TIMEOUT_MS` (default 500 ms), forward requests receive STOP until a fresh
result arrives. Choose this timeout according to the eventual sensor/inference
latency; an action must expire rather than persist forever after inference stops.

**Forward gating currently means commanded forward pitch, not measured forward
velocity.** The integrated IMU interface does not provide a velocity estimate. STOP
levels the drone; it does not implement active braking or guarantee a stationary
hover. UP adds collective thrust; it does not guarantee a climb or hold altitude.
Actual velocity gating, braking, and reliable obstacle clearance need a velocity/
position estimate and a navigation controller (and altitude feedback for UP).

`NPU.v` still contains no inference engine. It now exposes the action contract
while holding `available=0`, `action_valid=0`, `action=CONTINUE`. The production
connection in `Drone` is ready for that implementation. For simulation or a
separate development action source, set `Drone.USE_EXTERNAL_NPU=1` and drive
`npu_external_available`, `npu_external_action_valid`, and `npu_external_action`.
The default is zero: those external ports are ignored. External data must also
be synchronous to `clk`.

## Arming, validity, and failure behavior

`available` means the flight-controller implementation exists; it is high outside
reset. `command_valid` additionally requires a fresh calibrated IMU, `rc_valid`,
`motor_ready`, and a recent control tick. `Drone.flight_command_valid` routes this
health flag into `PeripheralInterface.motor_command_valid`; the peripheral also
gates it with radio freshness before driving the ESCs.

To arm:

1. Wait for ESC startup, healthy IMU samples, radio reception, and control ticks.
2. Publish a complete radio packet with the arm switch OFF.
3. Set thrust to zero and publish a complete packet with the arm switch ON.
4. Raise thrust after `motors_armed` becomes high.

The default `ARM_MAX_THROTTLE=0` requires exactly zero thrust when arming.
An arm-on packet at higher thrust consumes the arm permission; lowering thrust
with the arm switch still ON does not subsequently arm. Reset, radio loss, IMU
health loss/timeout, ESC not ready, and a stopped control tick all disarm and
require a new healthy OFF/ON packet sequence. The control watchdog counts `clk`
cycles independently of `control_tick` and defaults to 10 ms. It cannot detect a
failure of `clk` itself. This prototype's fault response is motor stop, not a
controlled airborne landing.

Throttles are immediately gated to zero when command health or the arm request
drops. Registered commands and telemetry clear on the next `clk`. An ESC packet
already transmitting completes; `MotorControl` sends stop on its next packet.
See the [motor interface guide](../PeripheralInterface/MotorControl/README.md)
for the startup interval and DShot cadence.

## Stabilization, mixer, and parameters

All gains use integer arithmetic with a scale factor of 256:

```text
roll correction  = (-roll * ATTITUDE_KP - roll_rate * RATE_KD) / 256
pitch correction = ((pitch_target - pitch) * ATTITUDE_KP - pitch_rate * RATE_KD) / 256
yaw correction   = ((yaw_rate_target - yaw_rate) * YAW_KP) / 256
```

Each correction is limited to +/-`MAX_CORRECTION` (default 400 DShot counts).
For nonzero thrust, collective DShot is `48 + thrust * 1999 / 1000`.
The X-quad mixer assumes these spin directions **viewed from above**:

| Position | Assumed spin | Mixed DShot command |
| --- | --- | --- |
| Front right | CCW | collective - roll + pitch + yaw |
| Front left | CW | collective + roll + pitch - yaw |
| Back right | CW | collective - roll - pitch - yaw |
| Back left | CCW | collective + roll - pitch + yaw |

Verify that wiring, physical motor positions, and propeller/spin directions agree
with these assumptions before motor testing. Change the mixer signs if the
airframe's rotation pattern differs. Outputs are clipped independently to
48..2047; thrust zero overrides all corrections with stop. Reserved DShot values
1..47 are never emitted. Clipping can reduce attitude-control authority; this
prototype does not implement mixer desaturation or thrust compensation for tilt.

Defaults are interface-development values, not gains tuned for flight. Configure
the standalone module's parameters, or `Drone`'s corresponding `FC_*` parameters,
for all timeouts, arming threshold, pitch deadband/limit, yaw limit, divert rate/
thrust increment, gains, and correction limit. `Drone` forwards `CLK_FREQ` too.
Observable outputs include `imu_fresh`, `npu_fresh`, `avoidance_active`,
`applied_action`, the three setpoints, and all four DShot throttle values.

## Verification

Run `./Run-Tests.ps1 -Module FlightController` for arming, ignored pilot roll,
forward/reverse/deadband pitch, yaw, attitude/rate feedback signs, atomic IMU
sampling, all five actions, invalid action handling, NPU gating/expiry/recovery,
thrust/motor saturation, exact watchdog boundaries, faults, explicit rearming,
and reset. `./Run-Tests.ps1 -Module Drone` also sends real serial radio packets
and decodes all four ESC outputs for forward flight, STOP, and fault responses.
It also exercises the default physical SPI path through sensor initialization,
calibration, arming, gyro-based stabilization, stale data, reset/rearm and sensor
disconnect. `./Run-Tests.ps1 -Module '*imu*'` runs the three sensor/estimator benches.
The full suite checks the existing peripheral and system behavior too.

These tests verify digital behavior. There is no simulated airframe dynamics,
Vivado timing result, physical IMU, or flight validation in this prototype.
