# Radio receiver

`RadioReceiver` implements the FS-iA6B servo iBUS interface described below.
Its helpers `IBusDecoder` and `RCMapper` live in this folder. `UartReceiver` is
shared with the lidar and lives in `Drone/PeripheralInterface/UartReceiver.v`.
`PeripheralInterface` instantiates this module;
`Drone` instantiates `PeripheralInterface`. Add all `.v` files under `Drone/`
recursively to Vivado as design sources. Do not add test files as design sources.

All logic uses `clk`; `rst` is synchronous and active high and must be asserted
before use. Synchronize any external reset button before this boundary.

## Outputs

| Output | Meaning |
| --- | --- |
| `throttle[9:0]` | Unsigned 0 to 1000 |
| `roll_command`, `pitch_command`, `yaw_command` | Signed 11-bit values, -500 to +500 |
| `arm_request` | Pilot request; the flight controller owns arming decisions |
| `avoidance_enable` | Pilot mode-switch request |
| `frame_valid` | One clock pulse per accepted frame |
| `rc_valid` | A checksum-valid frame arrived within the timeout |
| `channels[223:0]` | Fourteen raw values; CH1 is `[15:0]`, CH14 is `[223:208]` |

All channel values commit together after checksum validation. Commands reflect the
committed frame during `frame_valid`, allowing capture with `if (frame_valid)` at
the next rising edge. Watch `rc_valid` between frames as well: timeout does not
produce a `frame_valid` pulse. On timeout commands become zero and both switch
requests become false; raw channels retain the last accepted frame. Reset also
clears raw channels.

Default assignments: CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5 arm, CH6 mode.
Verify these on the transmitter. Override `ROLL_CHANNEL`, `PITCH_CHANNEL`,
`THROTTLE_CHANNEL`, `YAW_CHANNEL`, `ARM_CHANNEL`, and `MODE_CHANNEL` using one-based
numbers from 1 to 14. `SWITCH_THRESHOLD` defaults to 1500; at or above is true.
Controls clamp raw values to 1000 through 2000 before subtracting 1500 for centered
axes or 1000 for throttle. These outputs are stick commands, not angles or rates.
Every configuration parameter is forwarded through `PeripheralInterface` and `Drone`.

## Protocol

Defaults: `CLK_FREQ=100_000_000`, `BAUD_RATE=115_200`, `TIMEOUT_MS=100`,
`BYTE_TIMEOUT_US=500`. The idle-high, non-inverted 8N1 UART has a two-flop input
synchronizer and a rounded 868-clock bit period. It does not use `TickGenerator`,
which requires an exact divisor.

The 32-byte packet consists of `20 40`, 28 channel bytes, and a little-endian
checksum equal to `0xFFFF - sum(bytes 0..29)`. Checksum failures do not commit
channels or refresh validity. A framing error or a byte gap of 500 us abandons a
partial packet. This supports the standard fourteen 16-bit channel layout, not
extended-channel variants or sensor telemetry. See the
[Betaflight iBUS implementation](https://github.com/betaflight/betaflight/blob/master/src/main/rx/ibus.c).

`rc_valid` measures serial packet freshness, not the RF connection. The receiver
may continue emitting valid packets containing configured failsafe or held values.
Configure failsafe with throttle low and arm off, and verify receiver behavior.
See [FlySky's failsafe explanation](https://www.flysky-cn.com/journal/2021/2/21/about-the-flysky-remote-control-failsafe-function).

## Wiring and constraints

Use the servo iBUS output in iBUS mode, connect it to Pmod JA1, and share ground.
Power the receiver from the regulated 5 V supply in the root reference. Verify or
translate the signal to 3.3 V before connecting to the FPGA.

Merge these into the board top-level XDC, adapting port names and avoiding a
duplicate clock definition. Pin assignments are from
[Digilent's Basys3 master XDC](https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc).

```tcl
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -period 10.000 -name sys_clk [get_ports clk]
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports ibus_rx]
```

Constrain the asynchronous input only to the first synchronizer stage when adding
timing exceptions. Retain normal timing between the stages; both registers carry
`ASYNC_REG`. The `Drone` development top exposes internal commands for observation;
a final board design must route them internally or assign appropriate external I/O.

## Tests

From the repository root: `./Run-Tests.ps1 -Module RadioReceiver`.
The matching bench is
`tests/PeripheralInterface/RadioReceiver/RadioReceiver_tb.v`; it also works
with the VS Code Verilog runner from that file. It generates `RadioReceiver.vcd`
beside itself. For full internal waveforms, run `vvp RadioReceiver_tb.v.out +full_wave`
from its directory. Component benches share the same mirrored test folder.

The radio integration bench uses the actual 100 MHz clock and independently timed
serial input. It covers all 14 channels, mapping, clamping, invalid packets, reset,
timeout, line errors, and roughly 2% baud variation. Unit benches additionally
exercise all 256 UART byte values, every 16-bit mapper input, and the exact decoder
timeout boundary. Vivado synthesis, timing closure, and physical receiver behavior
remain to be verified in the integrated hardware design.
