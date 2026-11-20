# Drone parts and hardware reference

Updated: November 13, 2026.

**Latest wiring:** see [pins.md](pins.md) and the
[new battery/lights test](vivado/battery%20lights%20flight%20test/README.md).
Radio moved to JC1; ESCs to JB1-JB4; lights use JB7-JB9; SW15 arms;
Radio CH9/SwC selects headlights/landing lights; CH8/SwB enables blinking prop lights.
SW4..SW6 are unused. CH7/SwA and CH10/SwD are AI/hover requests (controllers inactive). Ultrasonic trigger/
divided echo are JC3/JC4, lidar C/R and D/T are JC7/JC8. The older wiring table below is retained as
historical context for the previous IMU flight test. Confirmed motor channels 0/1/2/3 are back-left/front-left/back-right/front-right.
Rotation after rewiring still needs verification. Battery sense uses JXADC1/JXADC7 with a confirmed 15k/1k (1:16) divider and grounded negative input.

This inventory records the user's parts-list image, hardware reports, and the
current repository wiring. Purchase links below were supplied by the user;
short Amazon links are transcribed from the image and have not all been resolved
or independently verified. Specifications described as listed are product-list
information, not measurements of the installed hardware. Quantities are stated
only where known.

## Airframe and propulsion

| Part | Model / quantity | Known details | Purchase link |
| --- | --- | --- | --- |
| Frame | YoungRC F450 Drone Frame Kit | F450 frame kit; actual assembled weight and remaining payload allowance have not been measured in this session. | [User's frame listing](https://a.co/d/0gjRRDdk) |
| Motors | A2212 1400KV Brushless Motor 10T; 4 motors | Brushless motors; exact manufacturer, current limits, and thrust/current data for the installed props remain unverified. | [User's motor listing](https://a.co/d/0i9Z66q8) |
| Propellers | Two-blade 9×4.7; purchased as 5 pairs | Verify handedness against actual rotation. Do not infer motor loading from prop size alone. | [User's propeller listing](https://a.co/d/0cL2IRDO) |
| ESC | AERO SELFIE 45A 4-in-1 ESC | Four motor channels. Current FPGA test sends DShot300, with separate throttle values for each channel. The 45A figure is the listed rating; operating limits depend on the exact board and cooling. | [User's ESC listing](https://a.co/d/0af9kiA7) |

ESC wiring reference: [manufacturer's connection diagram](https://aeroselfie.myshopify.com/cdn/shop/files/45A_4in1_ESC_Connection_Diagram.jpg).
Identify ESC channel labels on the board; physical motor positions below are the
confirmed project mapping, not an assumed connector order.

## Battery, charging, and power

| Part | Model / quantity | Known details | Purchase link |
| --- | --- | --- | --- |
| Installed flight battery | OVONIC Car-series soft-case 3S LiPo, 11.1V nominal, 5200mAh, listed 80C; 1 pack | Arrived and in use, confirmed November 13, 2026. Deans/T-plug main connector; JST-XHR-4P balance connector. 12.6V fully charged; nominal energy 57.72Wh. Published dimensions about 135 x 42 x 28mm; published weight conflicts at 340-358g, not measured. About 2.36 times the original nominal capacity, roughly 160-180g heavier; runtime gain depends on flight load/weight. | [Installed battery listing](https://www.amazon.com/dp/B09CTSFH36) |
| Original flight battery | OVONIC 3S LiPo, 11.1V nominal, 2200mAh, listed 50C | Deans/T-plug main connector; four-pin balance connector. Standard 3S LiPo full-charge voltage is 12.6V. **User reported swelling: retired from use and charging; replaced by the 5200mAh pack below.** | [Direct battery listing](https://www.amazon.com/dp/B08CZF373Y); [parts-list short link](https://a.co/d/0iSvZewZ) |
| Existing charger | HTRC 2S/3S LiPo balance charger | Listing specifies 100–240V AC input, 800mA balance-charge current, maximum output current 1000mA. Red = charging, green = full; an unlit LED is not explained in the listing. User observed red/green/off with the swollen pack; charger condition remains unresolved. | [Direct charger listing](https://www.amazon.com/dp/B0H4LC2J37); [parts-list short link](https://a.co/d/0gEH572o) |
| 5V regulators | Buck converter modules, listed 5–30V input to 5V/3A output; purchased as 6 modules | Regulates the flight battery down to 5V for Basys3 external power. Exact model, continuous-current capability, ripple, and wiring protection are unverified. | [User's regulator listing](https://a.co/d/0fEhI8Aj) |
| Battery connectors | T-plug male/female connectors; purchased as 3 pairs | Installed replacement battery retains Deans/T-plug compatibility. Check polarity and use properly assembled, insulated connections. | [User's battery-connector listing](https://a.co/d/0gehUwsI) |
| Motor connectors | Gold-plated 3.5mm bullet connectors; purchased as 20 pairs | Three motor-phase connections per motor. Earlier temporary taped contacts were reported; final connection quality has not been confirmed. | [User's motor-connector listing](https://a.co/d/08o9IiZ6) |

### Power arrangement used in testing

- ESC main power receives the flight battery voltage directly (previously reported around 11V).
- Basys3 external power receives regulated **5V**, not raw battery voltage.
- The IMU breakout currently receives **3.3V from Basys3**, not the 5V regulator.
- Basys3 ground, IMU ground, and ESC signal ground must share a reference. Maintain this reference when changing between external and USB board power.
- Keep battery voltage and ESC power/current-sense signals away from FPGA GPIO.
- Disconnect the flight battery from the drone before charging. Basys3 can be powered separately through USB for electronics work.

**Battery status:** On November 10 the user reported visible puffing/bulging,
reduced motor power, prolonged charging, approximately 12V total, and one red,
one green, and one unlit charger LED. These observations do not identify the
failure cause or establish individual-cell health. Do not charge, fly, or run
motors from that pack. Arrange disposal through a facility accepting damaged
LiPo batteries; do not place it in household trash or ordinary recycling.

Reference: [LiPo handling and safety instructions](https://apps.associatedelectrics.com/ateamapps/reedy_power/LiPo-warnings/index.html).

### Replacement installed

The OVONIC 5200mAh 3S 80C Deans/T-plug pack above is now installed.
Earlier 3000-3300mAh and 4500mAh suggestions were alternatives, not purchased
parts. No replacement charger or main-power relay has been confirmed purchased.
The existing HTRC charger's unusual LED indication remains unresolved.
The new capacity does not change the 3S voltage scaling or XADC divider.

## Control electronics and sensors

| Part | Model | Known details / project use | Source |
| --- | --- | --- | --- |
| FPGA board | Digilent Basys3 | Artix-7 XC7A35T-1CPG236C device family/package (`xc7a35tcpg236-1` in Vivado), 100MHz onboard oscillator, 16 switches, 16 LEDs, four-digit seven-segment display. Supplied by the stock room. Current test uses onboard display/switches and JA/JB headers. | [Digilent master pin constraints](https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc) |
| Transmitter / receiver | Parts list: FlySky FS-i6X transmitter + FS-iA6B receiver | User later identified the transmitter as **FS-i6**. Exact installed transmitter variant remains to be checked. Current design receives servo iBUS TX at 115200 baud and uses throttle, roll, pitch, and yaw; SW15 handles arming in the latest test rather than radio CH5 (older test used SW4). | [User's radio listing](https://a.co/d/0c1o97in) |
| IMU breakout | SparkFun ICM-20948 breakout, SEN-15335 | Accelerometer, gyroscope, and magnetometer hardware. Current controller uses accelerometer/gyro only over SPI; magnetometer is unused. Power from Basys3 3.3V. | [DigiKey part 15335](https://www.digikey.com/en/products/detail/sparkfun-electronics/15335/10279707) |
| LiDAR / ToF array | DFRobot SEN0628, 8×8 matrix ToF | Shared project driver supports 8×8 ranging over 115200-baud UART. **Excluded from the current flight-test top.** Header pins are not assigned in that test. | [DigiKey SEN0628](https://www.digikey.com/en/products/detail/dfrobot/SEN0628/28531125); [manufacturer UART documentation](https://wiki.dfrobot.com/sen0628/docs/21559) |
| Remote ID module | Ruko R111S | Listed separately as an out-of-pocket purchase. Power, installation, configuration, and operational status have not been confirmed here; no FPGA interface is documented. | [User's Remote ID listing](https://a.co/d/0apPtXpik) |

The NPU is a project logic subsystem rather than a separately identified
purchased part. It is also excluded from the current flight-test top.

## Current flight-test wiring

Source of truth: [flight-test guide](vivado/imu%20flight%20test/README.md) and the
test's constraints. Earlier radio-only and motor-only tests used different
assignments; this table records the current integrated test.

| Function | Basys3 connection | FPGA package pin / note |
| --- | --- | --- |
| Receiver servo iBUS TX | JB1 | A14 |
| IMU active-low CS (`/CS`) | JA1 | J1 |
| IMU MISO | JA2 | L2 |
| IMU SCLK | JA3 | J2 |
| IMU MOSI | JA4 | G2 |
| Back-left ESC signal | JA7 | H1; selected by SW0 |
| Front-left ESC signal | JA8 | K2; selected by SW1 |
| Back-right ESC signal | JA9 | H2; selected by SW2 |
| Front-right ESC signal | JA10 | G3; selected by SW3 |
| Arm request | Onboard SW4 | W15 |
| IMU VCC | Basys3 3.3V | JA6 or JA12 |
| Signal ground | Common GND | JA5 or JA11 are available ground pins |

Confirmed rotation viewed from above: **front-right/back-left clockwise;
front-left/back-right counterclockwise**. Switches SW0–SW3 are inclusive,
so multiple motors can be selected together. Throttle below 10 sends stop.

## Current software limitations relevant to the hardware

- No battery-voltage sensing, low-voltage warning, or voltage compensation is implemented in the flight test. Battery protection cannot be assumed from the FPGA code.
- The displayed number is radio throttle, not measured motor speed, thrust, or battery state.
- The estimator calibrates automatically while the IMU is still and level. Shared code currently uses ±2g acceleration and ±250°/s gyro ranges.
- The selected IMU mounting profile expects X forward, Y left, Z up. Level raw Z near +16384 was observed; full physical axis/correction-sign verification remains outstanding.
- Control gains and motor minimum-running commands have not been flight-validated. A new battery does not resolve possible controller-induced motor reductions, incorrect correction signs, or bad motor connections.

## Related project documentation

- [Main project README](README.md)
- [Integrated flight-test setup and diagnostics](vivado/imu%20flight%20test/README.md)
- [Flight controller](Drone/FlightController/README.md)
- [Motor control and DShot](Drone/PeripheralInterface/MotorControl/README.md)
- [IMU integration and calibration](Drone/PeripheralInterface/IMU/README.md)
- [Radio receiver](Drone/PeripheralInterface/RadioReceiver/README.md)
- [LiDAR array](Drone/PeripheralInterface/LidarArray/README.md)
