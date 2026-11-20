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
LD10: valid ultrasonic range solid; no/invalid echo blinking.
LD11: RC CH7 SwA AI override request; AI inactive.
LD12: RC CH10 SwD hold/hover request; hover inactive. SW11 X-position cm / SW12 Y-position cm diagnostic views; SW7 throttle view; SW8 IMU roll view; SW9 lidar centre mm view; SW10 ultrasonic mm view; center button resets.

Lidar C/R -> JC7/L17 (FPGA TX); D/T -> JC8/M19 (FPGA RX). UART mode.
Depth frames -> Basys3 USB serial TX/A18, 115200 baud.

Active diagnostic ultrasonic: trigger JC3, divided echo JC4; no hover control.
Display priority: configuration, SW14 errors, SW10 ultrasonic mm, SW9 lidar centre mm, SW8 roll, SW13 battery, SW7 throttle, default ready.

Default display: Arnd if actually armed, rdy if currently permitted to arm; CnFG during IMU setup. Selected views override default.

November 15 RC light update: SW4..SW6 unused. CH8=prop enable; CH9=off/head/head+landing.
LD11/LD12 show requests only; no ESC-health indicator exists. Other FPGA switches unchanged.

November 18 profile: accelerometer clipping is tolerated (gyro-only update); critical IMU faults retained. Relative yaw hold enabled. SW11/12 position diagnostics never influence motors.
