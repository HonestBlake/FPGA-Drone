# Transmitter controls

Recorded November 15, 2026. These are the requested physical transmitter
controls, distinct from the Basys3 SW0..SW15 switches. The bench integration uses CH7..CH10 as below. AI/hover mode requests are
observable, but their control paths remain inactive.

| Transmitter control | Requested function | Positions / behavior |
| --- | --- | --- |
| SwA | Enable AI override | OFF: no AI override permission. ON: permit AI intervention when its integration and health checks are implemented. This does not arm the motors. |
| SwB | Prop lights | OFF: disabled. ON: enable the existing blinking prop lights. |
| SwC | Headlights and landing lights | OFF: both off. First active position: headlights only. Second active position: headlights and landing lights. |
| SwD | Automatic hold / hover request | OFF: manual mode. ON: request hold/hover once its integration and sensor checks are implemented. This does not arm the motors. |

SwC is treated as three positions total: OFF plus the two requested active
steps. Verify physical direction and received values before connecting it.

## User-reported channel allocation

CH1..CH4 remain roll, pitch, throttle and yaw. The user specified:

| Channel | Source | Planned function |
| --- | --- | --- |
| CH7 | SwA | AI override permission |
| CH8 | SwB | Blinking prop lights |
| CH9 | SwC | OFF / headlights / headlights plus landing lights |
| CH10 | SwD | Hold/hover request |

Enable ten transmitted channels for this allocation. CH5/CH6 are not reassigned
by this plan. Actual received switch positions and polarity remain unverified.
The decoder's fourteen iBUS slots do not imply fourteen independent controls.

The parts list calls the transmitter FS-i6X; the user previously identified it
as FS-i6. Confirm the available auxiliary-channel menu before implementation.
FlySky lists the FS-i6X as 6-10 channels, default 6:
https://www.flysky-cn.com/i6x-canshu-1

The user subsequently reported an Aux Switches menu with channel count up to
10 and per-switch enable controls, consistent with FS-i6X-style configuration.
Enable SwA/B/C/D, select 10 channels,
and hold CANCEL to save. Assign CH7=SwA, CH8=SwB, CH9=SwC, CH10=SwD in
Functions setup / Aux. channels. Confirm the assignments on the transmitter's
channel display before enabling any FPGA functions. Menu capability is now
reported; assignments above are user-reported and actual received values remain unverified.
Manufacturer manual, sections 5.5 and 7.13:
https://static1.squarespace.com/static/645c2a3f23047437485c5189/t/66721c6a3a5adf21efbe0167/1718754436361/FS-i6X%2BUser%2Bmanual%2B20160817.pdf

## Bench integration - November 15, 2026

- CH8 SwB enables blinking prop lights on JB9; LD4 follows actual blinking output.
- CH9 SwC selects OFF / headlights (JB7) / headlights plus landing lights (JB8).
  LD6 follows headlights and LD5 follows landing lights.
- CH7 SwA is AI override permission, shown on LD11; it cannot affect motors yet.
- CH10 SwD is hold/hover request, shown on LD12; it cannot affect motors yet.
- SwA and SwD indicators show requests, not active AI/hover operation.
- Basys3 SW4/5/6 no longer control lights. Arming remains SW15 with all existing
  interlocks, motor selection remains SW0..3 and display views are unchanged.
- Two-position switches turn on at >=1750. SwC uses <1250 OFF, 1250..1749
  headlights, >=1750 both. Auxiliary values outside 900..2100 are rejected.
- Reset and loss of valid iBUS traffic clear radio lights and mode requests.
  This is serial freshness only: receiver RF failsafe may keep emitting valid
  packets with held values. Configure/verify auxiliary failsafe OFF.
- Shared radio mapper's legacy CH5 arm/CH6 mode outputs are still unused here.

Verify physical switch direction/endpoints, especially all three SwC positions,
before relying on the controls. The decoder tests use representative values
1000/1500/2000; actual installed transmitter values are not yet observed.


## Active ultrasonic hover update ? November 20, 2026

SwD / CH10 now captures current ultrasonic height when armed, all four motor
channels selected, throttle >=100/1000, and sensing/attitude are healthy. OFF
keeps height hold until the throttle stick catches the current collective, then
blends to manual over 500ms. Low stick alone does not cancel active height hold.
SW15 remains immediate disarm. Sensor failure freezes collective and flags a
fault while allowing manual pickup; RC/IMU failure retains the existing stop
policy. LD12 is solid for healthy hold, blinking for pickup/blend/degraded or
rejected request, and off in manual. SW10 reads sensor distance in millimetres.
See [height hold](Drone/FlightController/HOVER.md) for limits and recovery.
Earlier request-only descriptions of SwD/LD12 are superseded by this section.


## Received axis monitoring - November 22, 2026

Onboard SW7 selects yaw / CH4, SW11 roll / CH1, and SW12 pitch / CH2 in the
current hover test. Display is signed -500..+500, neutral zero, calculated as
clamp(channel,1000,2000)-1500 before flight-controller deadbands or scaling.
Positive values have a blank sign; negative values show a minus. Radio timeout
shows dashes. Select one view at a time; error/configuration retain priority.
Record centered readings and sustained correction inputs separately. No new
trim, mixer, motor, hover or NPU override behavior is introduced by these views.

## Yaw feedback monitoring - November 23, 2026

Onboard SW4 now shows measured yaw rate in degrees/second (tenths through
99.9, then whole degrees/second with no decimal point). SW5 shows requested
yaw correction in signed motor-command counts before individual motor
clipping; SW6 shows relative heading error in degrees to 0.1, limited to
+/-30.0. Positive means clockwise body rotation/correction, viewed from above.
SW7 remains raw received yaw input. No gains or trim offsets are changed.

SW4 works disarmed with a healthy IMU. With props removed and SW15 OFF, check
stillness near zero and the signs during gentle clockwise/counterclockwise
hand rotation. SW5/SW6 require a healthy controller command path, and remain
zero while disarmed or at throttle cutoff. A deliberate yaw-stick command
outside +/-20 resets the heading error. Unavailable data shows dashes.
Configuration and SW14 override these views; then priority is SW4, SW5, SW6,
followed by the earlier views. Select only one at a time.

Later November 23 gain trial: the bench heading gain increases from 8 to 16.
At zero yaw rate and the same +/-30deg heading bound, SW5 can show +/-187
heading correction counts (previously +/-93). Rate gain and pilot scaling are
unchanged. The combined correction limit remains +/-400. Physical behavior
still requires testing; this does not add a constant radio or motor trim.

Later November 23 pitch diagnostics: hold onboard SW8 ON and select SW4 for
measured pitch angle (degrees to 0.1), SW5 for requested pitch correction
(signed motor-command counts), or SW6 for pitch rate (deg/s). Positive angle,
rate and correction mean nose-up. With SW8 OFF, the existing yaw views remain.
SW8 alone stays IMU roll; SW12 alone stays raw RC pitch. Angle/rate can be read
disarmed, correction reads zero while stopped. No control gain/trim change.
