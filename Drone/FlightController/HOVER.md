# Experimental ultrasonic height hold

`HoverController.v` is a helper in the flight-computer subsystem. It is
**not instantiated by `FlightController`, `Drone`, or any uploaded test**.
The current board image and its SW15 arming remain unchanged. No existing
switch silently activates this controller.

The module produces a collective **RC throttle command in 0..1000 units**,
before attitude mixing and DShot conversion. It does not drive an ESC, arm
motors, or maintain horizontal position. The existing IMU controller must still
stabilize attitude. Height refers to distance above the reflecting surface, not
altitude above sea level.

## Interface and engagement

All inputs must be synchronous to `clk`; reset is synchronous active high.
Connect the paired Ultrasonic driver's `distance_mm`, `sample_valid`, and `valid`
to the corresponding distance/sample/range inputs. Supply actual armed status,
control/radio validity, and an `attitude_ok` decision from a verified IMU/tilt
policy. Do not tie health inputs high in a flying integration.

Until a mode control is deliberately assigned, leave `enable=0`. There must be
an observed OFF state after reset, followed by a rising enable edge with healthy
inputs, fresh distance inside 80..2000mm, and manual throttle >=100. The module
captures that height as its fixed target and the current throttle as its base;
initial output equals manual throttle, avoiding a step on entry. Raising or
lowering the non-idle stick does not change the target/base while holding.
Disable to regain manual control and re-enable to capture a new target.

Only use `throttle_out` as a future FlightController collective input when the
integration's hover policy permits it. `active` is the explicit indication that
height hold is controlling it; otherwise `throttle_out` passes manual throttle
through, clamped to 0..1000. Reset produces zero. Keep FlightController's own
arming, radio/IMU health checks, and throttle-stop gates intact.

## Control law and limits

Each new distance updates a bounded proportional/vertical-rate damping command:

```text
error_mm = target_height_mm - distance_mm
vertical_speed_mmps = (distance_mm - previous_mm) * 1000 / SAMPLE_PERIOD_MS
correction = (KP * error_mm - KD * vertical_speed_mmps) / 256
output = captured_manual_throttle + bounded_correction
```

Defaults: sample period 65ms, KP=16, KD=64, correction +/-150 throttle counts.
Output is clamped to 10..1000 while active. These gains are **provisional and not
flight tuned**. The fixed-period derivative must match the Ultrasonic driver's
65ms start-to-start schedule. Do not feed irregularly timed measurements or
change the sensor interval without changing the controller's parameter.
No integral term, hover-thrust learning, altitude setpoint adjustment, tilt
correction, or dynamic sensor fusion is implemented.

A measurement step exceeding 300mm, an out-of-window reading, missing/invalid
range, stale data (150ms), loss of arm/control/attitude permission, or pilot
throttle below 10 disengages holding. Validity/health/idle gates also suppress
`active` immediately between updates. After a rejection/dropout, a valid sample
alone cannot resume holding: switch enable OFF, then ON when healthy.
`fault` reports the rejected entry or dropout and clears when enable is OFF.

**Loss of height sensing falls back to manual throttle, not an automatic
landing or motor stop.** Integrating this behavior into an aircraft requires an
explicit, tested mode/failsafe policy. Ultrasonic reflections can be wrong even
within range; passing these guards does not prove height is correct. Begin with
a stationary range-display test, then verify sensing under motor noise before
ever enabling throttle corrections. A constrained Vivado build and physical
sign/gain tests remain required before flight.

## Verification

Run `./Run-Tests.ps1 -Module HoverController` from the root. The bench covers
boot-time enable inhibition, target/base capture, correction signs and limits,
throttle clamps, spike/out-of-range rejection, stale/invalid ranges, disarm,
control/tilt loss, idle override, required enable cycle, and reset.
