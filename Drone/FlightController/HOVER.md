# Ultrasonic height hold with throttle pickup

The battery/lights flight test enables `HoverController` using transmitter
SwD / CH10 (ON >=1750). This controller supplies collective throttle to the
existing attitude controller, before mixing and DShot. It cannot arm motors or
control horizontal position. The production `Drone` top does not enable this
bench feature automatically. NPU predictions remain display-only. The current hover image sets
`ENABLE_POSITION_DIAGNOSTICS=0` because the combined image exceeded FPGA logic
capacity. SW11/SW12 X/Y views are inactive; the estimator source remains available
for a separate build. This estimator never supplied motor-control feedback.

## Height and entry

The sensor and SW10 report millimetres. A ground reading of 120 means 120mm
from the sensor face to the reflecting surface, including the sensor mounting
height above the feet. No zero adjustment is needed to capture and maintain
that same measured distance. A change of 610mm corresponds to about two feet.

After boot, the mode switch must be observed OFF, then ON. Entry requires SW15
armed, all four motor channels selected, valid RC/IMU/control, attitude within
+/-15 degrees roll/pitch, a fresh ultrasonic reading within 80..2000mm, and
manual throttle >=100/1000. The target is the latest accepted height, and the
base throttle is the current pilot command. The first held command is identical
to the manual command, avoiding a step on entry. This is height capture, not
an automatic takeoff mode. Enable it after reaching the desired height.

## Hold and manual return

During hold, the throttle stick may be moved to zero without cancelling the
height loop. Turning SwD OFF enters **pickup**: height regulation continues
until the manual throttle equals or exceeds the current held collective. This
comparison follows the live controller command, not just its initial base.
There is no automatic timeout that drops a zero-position stick into control.

After pickup, collective interpolates from its current value to the live stick
over 500ms (configurable). Dropping the stick during that interval also ramps
the output, rather than causing an immediate cut. Once the blend completes,
normal manual throttle and the existing below-10 cutoff apply. Turning SwD ON
again during pickup or blending captures a new current height/base when range
and attitude are usable, including when the stick is at zero.

SW15 OFF remains immediate disarm, independent of hold, pickup, or blending.
RC/IMU/control loss still uses the existing flight-controller motor-stop and
explicit rearming policy. Radio health here means serial packet freshness; a
receiver RF failsafe may keep sending valid packets and must be checked separately.
Neither case retains an autonomous command for a
later rearm; a fresh SwD OFF/ON sequence is required to enter height hold again.

## Sensor failure and indicators

During hold or pending pickup, a missing, invalid or stale range (150ms), a sample
jump over 300mm, or attitude
outside the height-loop envelope stops height corrections and freezes the last
collective. It does **not** fall straight back to a low/zero throttle stick.
This frozen command is not altitude regulation or an automatic landing. Pilot
pickup remains available with SwD OFF; after correcting sensing, an OFF/ON
cycle can capture a fresh target. Invalid sensor data alone does not disarm.

LD12 is solid during healthy height hold, blinking during pickup, blending, or
a rejected/degraded hold request, and off in manual mode. LD10 still indicates
ultrasonic health. SW14 shows E031 for hover rejection/degraded control; existing
higher-priority faults, including ultrasonic E030, take precedence. SW10 remains
the distance view and SW9 remains the NPU prediction view.

## Control law and validation

The captured-base controller uses KP=16 and KD=64, divided by 256. Vertical rate
uses the paired driver's nominal 65ms sample interval, with a quarter-weight
update low-pass filter. Corrections are limited to +/-150 throttle counts and
active output to 10..1000. Gains are initial bench values, not flight tuned.
There is no integral trim, tilt compensation, position hold or terrain tracking.

The unit bench verifies captured target/base, correction signs, filtered rate,
zero-stick hold, live-command pickup, signed upward/downward blend, recapture,
range/spike/stale/tilt freeze, and health/disarm/reset. `HoverBench_tb` drives
physical ultrasonic echo pulses and checksummed CH10 iBUS packets, models the
SPI IMU and decodes all four ESC wires. It verifies that the effective collective
bypasses the manual-idle cutoff only during active hover/handover, while SW15 and
radio failure still stop motors. These are digital tests; actual gains and
sensor behavior under propeller vibration still require physical validation.
