jxadc1 -> battery shifted to 0-1v in

Planned transmitter SwA/SwB/SwC/SwD controls: see RC_CONTROLS.md.
User-reported radio mapping: CH7 SwA AI permission, CH8 SwB prop lights,
CH9 SwC headlights/landing lights, CH10 SwD hold/hover request.
Received physical values/polarity remain unverified. Bench integration uses the mapping below.

IMU:
blue -> bar cs
green -> miso
yellow -> sclk
orange -> mosi

blue -> ja1
green -> ja2
yellow -> ja3
orange -> ja4


IR:
not wired now

ESC:
4 channels: 0 back-left, 1 front-left, 2 back-right, 3 front-right
jb1-jb4

Ultra sonic:
trigger -> jc3
echo -> jc4

led singals
JB7 / RC CH9 SwC positions 1/2 -> headlights
JB8 / RC CH9 SwC position 2 -> landing lights
JB9 / RC CH8 SwB -> blinking prop lights
jb7-jb9
Radio receiver iBUS TX -> JC1

Test switches: SW15 arm; radio CH9 SwC headlights/JB7 and landing lights/JB8; SW13 battery view; SW14 error view; radio CH8 SwB prop lights/JB9.
Motor selectors: SW0/JB1 back-left; SW1/JB2 front-left; SW2/JB3 back-right; SW3/JB4 front-right. Positions confirmed; rotation directions still need checking after rewiring.
Battery sense: JXADC1/VAUX6P; matching JXADC7/VAUX6N must connect to common ground.
Confirmed divider: 15k upper / 1k lower (1:16); JXADC7 tied to common ground.

Current test onboard status LEDs (approximately 1 Hz blink):
LD0..LD3: motor 0..3 selected = solid, deselected = blinking.
LD15: armed solid; can arm but disarmed blinking; arming blocked off.
LD8: IMU calibrated/healthy = solid; setup/calibration/fault = blinking.
LD7: radio valid = solid; disconnected/invalid = blinking.
LD13: valid battery >20% solid; <=20% blinking; invalid/off.
LD14: error view selected solid; otherwise error blinking, no error off.
LD6: headlights on solid/off off; LD5: landing lights on solid/off off.
LD4: follows prop lights blinking when enabled; disabled off.
LD9: lidar recent complete scan solid; startup/error/stale blinking.
SW9 now displays the latest display-only NPU prediction: Cont / LEFt / rIGt / uP / StoP.
Confirmed model classes 0..4: Continue / Left / Right / Up / Stop. NPU cannot override motors.
No valid recent lidar/NPU result shows dashes. SW14, SW11/SW12 and SW10 have higher display priority.
LD10: valid ultrasonic range solid; no/invalid echo blinking.
LD11: RC CH7 SwA AI override request; AI inactive.
LD12: actual hover state (solid healthy hold; blink pickup/blend/degraded; off manual).
Views: SW7 RC yaw, SW11 RC roll, SW12 RC pitch, SW8 IMU roll, SW9 NPU command, SW10 ultrasonic mm; center button resets.

Lidar C/R -> JC7/L17 (FPGA TX); D/T -> JC8/M19 (FPGA RX). UART mode.
Depth frames -> Basys3 USB serial TX/A18, 115200 baud.

Active diagnostic ultrasonic: trigger JC3, divided echo JC4; no hover control.
Current display priority: configuration, SW14 errors, SW11 RC roll, SW12 RC pitch, SW10 ultrasonic mm, SW9 NPU command, SW8 IMU roll, SW13 battery, SW7 RC yaw, default status.

Default display: Arnd if actually armed, rdy if currently permitted to arm; CnFG during IMU setup. Selected views override default.

November 15 RC light update: SW4..SW6 unused. CH8=prop enable; CH9=off/head/head+landing.
LD11/LD12 show requests only; no ESC-health indicator exists. Other FPGA switches unchanged.

November 18 profile: accelerometer clipping is tolerated (gyro-only update); critical IMU faults retained. Relative yaw hold enabled. SW11/12 position diagnostics never influence motors.


Active ultrasonic height hold (November 20): transmitter SwD / CH10 ON captures
current height; OFF waits for throttle pickup, then blends to manual over 0.5s.
SW15 OFF still immediately disarms. Requires all four motor selectors and armed,
healthy inputs plus throttle >=100/1000 to enter. LD12: healthy hold solid,
pickup/blend/degraded/rejected request blinking, manual off. SW10 is distance mm.
Sensor failure freezes last collective until pickup or explicit re-entry; it
cannot maintain height without sensing. E031 hover fault, E030 sensor fault.
NPU remains display-only. Earlier hover request-only descriptions are superseded.

Hover-image resource profile: SW11/SW12 X/Y diagnostic views are inactive
(ENABLE_POSITION_DIAGNOSTICS=0). Their estimator source is retained. This frees
FPGA logic for hover alongside the NPU; attitude/yaw control is unchanged.

November 22 RC monitor update: default SW7=yaw CH4, SW11=roll CH1, SW12=pitch CH2.
Signed command counts -500..+500, channel center=0, minus shown for negative,
blank sign for positive. Invalid radio=dashes. These are pre-deadband readings;
no trim offset is applied. LD7/LD11/LD12 keep their existing status roles.
This supersedes the earlier inactive/position descriptions for SW11/SW12 in
the uploaded hover profile; the optional position-only build remains available.

November 23 yaw diagnostic views: SW4 measured IMU yaw speed (deg/s, tenths
through 99.9; whole units above that with decimal off), SW5 requested yaw
correction (signed motor-command counts before individual motor clipping),
SW6 relative heading error (degrees to 0.1, +/-30.0 cap). Positive is clockwise
viewed from above. SW7 remains received yaw command, SW11 roll, SW12 pitch.
Current priority: CnFG, SW14, SW4, SW5, SW6, then the earlier views in order.
SW4 can be checked while disarmed. SW5/SW6 require a healthy controller path
and read zero while disarmed or at throttle cutoff. Heading error resets for
deliberate yaw input outside +/-20. Unavailable feedback shows dashes.
SW4..SW6 are display selectors only; SW15 remains arm. Light controls remain
the transmitter SwB/SwC. No flight gains or trim offsets changed in that image.

Later November 23 yaw gain trial: only the bench heading gain changes 8 -> 16.
At zero yaw rate, +/-30.0deg error now contributes +/-187 correction counts
on SW5; combined heading/rate correction still clamps at +/-400. Switch/pin
assignments are unchanged. Earlier diagnostic-only gain statements describe
the previous image. No fixed trim is applied.

Later November 23 pitch diagnostic modifier: SW8 + SW4 = measured pitch angle
(deg to 0.1), SW8 + SW5 = pitch correction counts, SW8 + SW6 = pitch rate
(deg/s). Positive = nose-up for all three. Select only one of SW4/SW5/SW6.
SW8 OFF retains yaw views; SW8 alone remains roll and SW12 alone RC pitch.
No pins, motor behavior, yaw tuning or lighting assignments change.
