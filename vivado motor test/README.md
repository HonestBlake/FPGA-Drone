# Basys3 radio throttle and motor test

Standalone `RadioMotorTest` top with copied radio, display, and motor modules.
No flight controller, IMU, mixer, or radio arm channel is used. Target is the
Basys3 `xc7a35tcpg236-1` with its 100 MHz clock. The earlier `vivado test/`
project is preserved.

## Connections

| Connection | Basys3 pin | FPGA pin | Selection |
| --- | --- | --- | --- |
| Receiver servo iBUS TX | JB1 | A14 | Always received |
| ESC motor 1 signal | JA7 | H1 | SW0 |
| ESC motor 2 signal | JA8 | K2 | SW1 |
| ESC motor 3 signal | JA9 | H2 | SW2 |
| ESC motor 4 signal | JA10 | G3 | SW3 |

These are ESC signal inputs, not motor power connections. Share signal ground
between receiver, Basys3, and ESC. Use a 3.3 V compatible receiver signal and ESC
inputs. The receiver uses servo iBUS, 115200 baud, non-inverted 8N1; throttle
defaults to CH3. Motor numbers correspond to these four wires, regardless of
where the motors are mounted. JA5/JA11 are ground and JA6/JA12 are power; neither
is used as a motor signal. Pin assignments follow
[Digilent's master XDC](https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc).

## Operation

Keep propellers removed for this bench test.

1. Start with SW0-SW3 off and transmitter throttle down. Power the ESC before
   resetting the FPGA test with the center button.
2. Allow one second for the motor module's startup stop packets (LD5 turns on).
3. Turn on any combination of selectors: SW0 for motor 1, SW1 for motor 2, SW2 for
   motor 3, and SW3 for motor 4. All four on selects all four motors. Keep throttle
   below 10 until LD4 lights.
4. Raise throttle to drive every selected motor at the same command. Unselected
   motors receive DShot stop packets. Throttle 0-9 always sends motor stop;
   throttle 10 is the first nonzero command.
5. Lower throttle before changing selections. Any change of the switch mask stops
   the group and requires throttle below 10 again before the selected motors run.

No switches selected stops all motors. Radio packet timeout also stops all
motors and requires throttle below 10 again after reception resumes.
The center button resets reception, the enable interlock, and the one-second ESC
startup interval. A new selection at throttle 10 or above must first observe
throttle below 10 before it can run.
The startup timer is local timing, not acknowledgement from the ESC.

The seven-segment display always shows received throttle in **decimal 0000-1000**,
independent of motor selection. No recent valid packet shows **`----`**. Radio
timeout is 100 ms. This detects stale serial packets; a receiver that continues
sending its own held/failsafe values still counts as valid. Configure receiver
failsafe to throttle zero.

| LED | Meaning |
| --- | --- |
| LD0-LD3 | Selected motors (SW0-SW3; multiple LEDs may be on) |
| LD4 | Selected group enabled after observing throttle below 10 |
| LD5 | One-second startup stop interval complete |
| LD6 | Recent checksum-valid radio packet |
| LD7 | One blink per second while the test runs |

The reused motor modules send ordinary unidirectional **DShot300**, updated at
1 kHz. Throttle 0-9 maps to stop (0); values 10-1000 map as
`2 * throttle + 47`, giving DShot 67-2047. Values 1-47 are never sent. Every
selected motor receives this value. The display retains the received throttle,
so it can show 0001-0009 while the motors receive stop. See the
[DShot specification](https://betaflight.com/docs/development/API/Dshot).
A stop affects the next packet; a packet already being transmitted completes.

## Build and simulation

From the repository root:

```powershell
& '.\vivado motor test\Run-Test.ps1'
```

The simulation sends UART packets through the actual top, checks the physical
display pins, and decodes every motor output with DShot pulse-width and checksum
checks. It covers all 16 switch masks, startup with high throttle, low-throttle
interlocks, switch changes, cutoff at 0-9 and the first running value at 10,
maximum/clamped throttle, corrupt radio packets, timeout/recovery, and reset.
The board top is also compiled with its default parameters.

From this directory in a Vivado-enabled shell:

```powershell
vivado -mode batch -source build_bitstream.tcl
```

`create_project.tcl` creates the project without building it. The build script
creates it if needed, or resets its generated runs and rebuilds. Open
`build/radio_motor_test.xpr`. Top is `RadioMotorTest`; simulation top is
`RadioMotorTest_tb`. Bitstream is
`build/radio_motor_test.runs/impl_1/RadioMotorTest.bit`. Add only this project's
`src/*.v` as design sources, because copied module names match the originals.

The sources are independent snapshots of `RadioReceiver`, `IBusDecoder`,
`RCMapper`, `UartReceiver`, `MotorControl`, `MotorDriver`, `TickGenerator`, and
`SevenSegmentDisplay`. Only the display copy adds optional F-as-minus support
from the previous radio test. All receiver and motor modules are unchanged.

## Standalone startup

A JTAG upload alone is lost on power-off. The separate `program_flash.tcl`
script writes and verifies configuration flash and preserves its prior contents
in a timestamped backup under `build/`. It targets the previously identified
Macronix device on this Basys3 and checks the hardware target before writing.
Run only after building and checking the bitstream:

```powershell
vivado -mode batch -source program_flash.tcl
```

For automatic startup, turn power off and set **JP1 MODE to QSPI**, then power
on. For supply through external header J6, set the power-source jumper JP2 to
EXT. Flash boot and power-source selection are separate settings. See
[Digilent's board manual](https://reference.digilentinc.com/_media/reference/programmable-logic/basys-3/basys3_rm.pdf).

Simulation and Vivado timing checks verify the digital logic; actual motor
response still depends on ESC firmware, wiring, power, and electrical compatibility.

Verified with Icarus and Vivado 2026.1: the integration simulation passes,
synthesis/implementation and bitstream generation complete, all 100 MHz timing
constraints are met (setup slack 1.856 ns; hold slack 0.122 ns), and DRC reports
zero violations. Unused radio outputs generate synthesis warnings. Display,
status LED, and ESC outputs have no external capture clock, so Vivado reports
missing output-delay methodology warnings; all internal register paths remain
timed. Vectorless power analysis also warns about assumed reset activity.

The JB1 radio / throttle cutoff / multiple-selection update was programmed and
verified in the connected Basys3 flash on November 9, 2026. JP1 read as QSPI
mode (`001`); a hardware reboot loaded the updated image from flash and asserted
DONE. The prior 4 MB image is backed up at
`build/flash_before_motor_test_20261109_132805.bin`. This confirms configuration
from flash; physical motor spin has not been observed by the automated checks.
