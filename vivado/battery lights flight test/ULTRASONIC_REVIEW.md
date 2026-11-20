# Ultrasonic review - November 13, 2026

Reviewed the shared Ultrasonic module, board wrapper, JC1/JC3/JC4 constraints,
latest successful flash-program/boot log, and sensor documentation.

## Findings

- Receiver TX is JC1/K17; trigger JC3/N17; divided echo JC4/P18.
- Module clock parameter matches the wrapper 10MHz logic clock.
- Trigger is high for 10us, repeated every 65ms. Documentation specifies at
  least 10us and a measurement cycle longer than 60ms.
- Echo is synchronized through two registers. Its HIGH width (not the delay
  until rising) determines distance: ticks * 340000 / (2 * clock Hz), rounded mm.
- Accepted range is 20..4000mm; combined response/measurement timeout is 35ms.
- Two feet is 609.6mm, corresponding to about 3.586ms echo HIGH at 340m/s.
- Good measurements clear the ultrasonic fault automatically. No IMU or
  arming permission gates trigger output; no ultrasonic fault gates motors.
- First measurement starts immediately after reset, before sensor power-up
  settling is guaranteed. Periodic retries should recover after sensor startup;
  this alone does not explain a persistent fault after several seconds.
- All four distinct internal faults become E030 in the current display:
  1 no rising echo, 2 echo stuck high/timeout, 3 range rejected,
  4 echo already high before triggering. Thus E030 cannot locate the failure.

## Verification

Shared Ultrasonic regression passed again: pulse timing/spacing, boundary
ranges, asynchronous echo, missing/stuck echoes, recovery, disable/reset.
Additional board-wrapper audit at 10MHz passed: 10us trigger, simulated 612mm
range, seven-segment output, health, timeout, error-view priority, motor isolation.
This verifies digital behavior under simulated inputs, not the sensor waveform.
No production HDL or FPGA image was changed/uploaded during this review.

## Hardware interpretation

HC-SR04 uses 5V power and produces a 5V echo. Adafruit documents 3V or 5V trigger
compatibility; a 3.3V FPGA trigger is therefore not an obvious interface mismatch.
Use ECHO -> 2.2k -> sense/JC4 -> 3.3k -> ground, giving 3.0V from a 5V echo.

No specified normal unpowered VCC/GND resistance was found in the referenced
sensor documentation. A stable 50-ohm reading in both directions merits
investigation but is not a conclusive failure test. The module is active
semiconductor circuitry; do not calculate its actual 5V operating current from
an ohmmeter reading. Published operating current is about 15mA.

Best discriminator: independently verify supply voltage at the sensor, trigger
pulses at its actual pin, echo at the sensor output, and echo after the divider.
With a correct trigger and supply, no valid echo from a large flat target points
toward the sensor/target; a good sensor echo absent at JC4 points toward wiring
or the divider; a valid JC4 waveform with E030 points back toward FPGA capture.
A known-good Arduino test or replacement sensor can provide an independent check.

## Sources

- HC-SR04 manufacturer datasheet hosted by SparkFun:
  https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
- Adafruit sensor pinouts (3V/5V trigger, 5V echo):
  https://learn.adafruit.com/ultrasonic-sonar-distance-sensors/pinouts
- Adafruit divider wiring:
  https://learn.adafruit.com/distance-measurement-ultrasound-hcsr04/connect-the-sensor
