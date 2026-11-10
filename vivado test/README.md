# Basys3 radio receiver display test

Standalone copy of the radio receiver and seven-segment display with a new
`RadioDisplayTest` top. Nothing under `Drone/` is required by this project.
Target: Basys3, Artix-7 `xc7a35tcpg236-1`, onboard 100 MHz clock.

## Wiring and use

Connect the receiver's **servo iBUS TX output to JC1** (FPGA K17, top input `tx`)
and share ground. Use an idle-high, non-inverted, **3.3 V compatible** signal;
level-shift if needed. Power the receiver from its appropriate supply. The FPGA
input expects 115200-baud 8N1 servo iBUS packets, not the receiver's sensor port
or individual PWM channels. `tx` is an FPGA input named for the receiver output.

Turn on exactly one of SW0-SW3, leaving the other three off:

| Switch | Display | Default receiver channel |
| --- | --- | --- |
| SW0 | Throttle, decimal 0000 to 1000 | CH3 |
| SW1 | Roll, decimal -500 to 0500 | CH1 |
| SW2 | Pitch, decimal -500 to 0500 | CH2 |
| SW3 | Yaw, decimal -500 to 0500 | CH4 |

Values are the receiver module's mapped commands, not raw 1000-2000 channel
values. Positive values have leading zeros; negatives show a minus in the left
digit, e.g. `-001`. Decimal points stay off. With SW0-SW3 all off, the display
shows `----` even with no receiver data, so you can confirm the design is running.
Multiple selected switches, or a selected channel with no checksum-valid packet
within 100 ms, blanks the display. Changes
appear on the next full display sweep (normally within about 4 ms). SW4-SW15
are unused. The center button resets reception; a selected channel stays blank
until a new good packet arrives, while all switches off shows `----` after
reset is released. Startup automatically resets the design.

## Vivado

In Vivado's Tcl console, use the full path to this script, in braces for spaces:

```tcl
source {C:/Users/blake/Programing/School Projects/FPGA Lab/vivado test/create_project.tcl}
```

Or, from the repository root in a Vivado-enabled shell:

```powershell
vivado -mode batch -source "vivado test/create_project.tcl"
```

Open `build/radio_display_test.xpr` if created in batch mode. Run synthesis,
implementation, and Generate Bitstream; then use Hardware Manager to program
the Basys3. The script creates the project only; it does not program hardware.
It deliberately does not overwrite an existing project in `build/`.

For manual setup, add only `src/*.v` as design sources, select
`RadioDisplayTest` as top, and add `constraints/Basys3-RadioDisplayTest.xdc`.
Optionally add `sim/RadioDisplayTest_tb.v` as a simulation source and select
`RadioDisplayTest_tb` as simulation top. Do not add the repository's `Drone/`
sources alongside these copies because their module names are identical.
Use the default top parameters for the board; the testbench overrides timing
parameters only to speed up simulation.

Clock, JC1, SW0-SW3, center button, and all display pins are constrained using
[Digilent's Basys3 master XDC](https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc).
Only paths into the first stage of each asynchronous input synchronizer are
excluded from timing; the remaining logic stays timed at 100 MHz.

## Startup without the laptop

A normal JTAG upload loads volatile FPGA memory and is lost when power is
removed. For standalone startup, program the test image into configuration
flash, then power off and set the physical **JP1 MODE jumper to QSPI / SPI
Flash**. JP1 was read as JTAG mode on the connected board; software cannot move
this jumper. With flash programmed and JP1 in QSPI mode, the test loads at
power-on. Leave SW0-SW3 off to see `----` without receiver data.

The external power header is J6; its power-source jumper JP2 must select EXT
when supplying power there. Power-source selection and FPGA boot-mode selection
are separate. See [Digilent's board manual](https://reference.digilentinc.com/_media/reference/programmable-logic/basys-3/basys3_rm.pdf).

The connected board's flash identifies as Macronix JEDEC ID C2 20 16, supported
by Vivado as `mx25l3273f-spi-x1_x2_x4`. The flash image is
`build/RadioDisplayTest.mcs`, generated for 4 MB SPIx1 from the tested bitstream.
A full 4 MB backup of the original flash contents is saved as
`build/flash_before_radio_test_20261105_120626.bin`. These generated files are
ignored by Git. Rebuilding or uploading a new `.bit` file over JTAG does not
update the stored flash image; program flash again to change power-on behavior.

## Included sources and checks

`RadioReceiver.v`, `IBusDecoder.v`, and `RCMapper.v` are copied from
`Drone/PeripheralInterface/RadioReceiver/`; `UartReceiver.v` is copied from
`Drone/PeripheralInterface/`. `SevenSegmentDisplay.v` is copied from
`Drone/System/` with one optional `F_IS_MINUS` parameter added to render signed
decimal commands. Its default remains the original hexadecimal F glyph.
The receiver and display count their own timing, so `TickGenerator` and the
rest of `System` are not dependencies. Copies are independent snapshots; later
changes in `Drone/` do not propagate here automatically.

Run the standalone self-checking simulation from the repository root:

```powershell
& '.\vivado test\Run-Test.ps1'
```

It sends serial iBUS packets through the actual top and verifies physical
segment/anode outputs for all four selections, signed decimal values, limits,
zero, idle dashes before reception and after timeout/reset, invalid selections,
corrupt checksums, timeout, startup, reset, and
recovery. It also compiles the board top with its default parameters.

Verified with Icarus Verilog and Vivado 2026.1: simulation passes, synthesis and
implementation complete, bitstream generation succeeds, and routed 100 MHz
timing meets setup/hold. DRC reports no
violations. The generated project is `build/radio_display_test.xpr`; the
bitstream is `build/radio_display_test.runs/impl_1/RadioDisplayTest.bit`.
Generated build files are ignored by Git.

Vivado reports missing output-delay methodology warnings for the 11 display
pins, which drive an unclocked LED display and have no external capture clock.
Internal register paths remain timed. Unused receiver channel/status outputs
also produce synthesis warnings, and vectorless power analysis warns about its
assumed reset activity. Physical receiver and board operation still need to be
checked on hardware.
