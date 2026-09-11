# FS-iA6B iBUS receiver

`IBusReceiver` implements the data path in the root `ibus_basys3_interface.md`:

```text
ibus_rx -> UartReceiver -> IBusDecoder -> RCMapper -> flight computer
```

Add all four `.v` files in this directory as design sources in Vivado. Instantiate
`IBusReceiver` inside the flight-computer top level. Its outputs are internal FPGA
signals, not a separate serial connection to an external computer. Keep all logic
on the 100 MHz clock and assert synchronous, active-high `rst` before use. If reset
comes from a board button, synchronize it in the top level.

## Flight-computer interface

| Output | Meaning |
| --- | --- |
| `throttle[9:0]` | Unsigned 0 to 1000 |
| `roll_command`, `pitch_command`, `yaw_command` | Signed 11-bit values, -500 to +500 |
| `arm_request` | Pilot arm-switch request; the flight computer owns arming logic |
| `avoidance_enable` | Pilot mode-switch request |
| `frame_valid` | One 100 MHz clock pulse per accepted frame |
| `rc_valid` | A checksum-valid frame has arrived within the timeout |
| `channels[223:0]` | Fourteen raw 16-bit values; CH1 is `[15:0]`, CH14 is `[223:208]` |

All raw channels commit together after checksum validation. Mapped commands reflect
that committed frame while `frame_valid` is high, so downstream logic can capture
them with `if (frame_valid)` at the next rising edge. Watch `rc_valid` every clock,
including between frames: it falls on timeout without a `frame_valid` pulse.
On reset or timeout the mapped controls become zero and both switch requests are
false. Raw channels retain the last accepted frame on timeout and clear on reset.

```verilog
wire [9:0] throttle;
wire signed [10:0] roll_command, pitch_command, yaw_command;
wire arm_request, avoidance_enable, frame_valid, rc_valid;

IBusReceiver radio (
    .clk(clk), .rst(rst), .ibus_rx(ibus_rx),
    .throttle(throttle), .roll_command(roll_command),
    .pitch_command(pitch_command), .yaw_command(yaw_command),
    .arm_request(arm_request), .avoidance_enable(avoidance_enable),
    .frame_valid(frame_valid), .rc_valid(rc_valid), .channels()
);
```

The default channel assignments are CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw,
CH5 arm, and CH6 avoidance mode. Verify these on your transmitter. Override
`ROLL_CHANNEL`, `PITCH_CHANNEL`, `THROTTLE_CHANNEL`, `YAW_CHANNEL`, `ARM_CHANNEL`,
and `MODE_CHANNEL` with one-based channel numbers from 1 to 14 as needed.
`SWITCH_THRESHOLD` defaults to 1500; switch values at or above it are true.
Centered controls subtract 1500 and throttle subtracts 1000 after clamping raw
values to 1000 through 2000. Values are stick commands, not angles or angular rates.

## Serial protocol and timeouts

Defaults: `CLK_FREQ=100_000_000`, `BAUD_RATE=115_200`, `TIMEOUT_MS=100`, and
`BYTE_TIMEOUT_US=500`. The UART uses a two-flop input synchronizer and samples at
bit centers. Its rounded divider uses 868 clocks per bit at the default settings.
It does not use `TickGenerator`, which requires an exact frequency divisor.

The decoder accepts the 32-byte servo packet: `20 40`, 28 channel bytes,
and a little-endian checksum equal to `0xFFFF - sum(bytes 0..29)`. Invalid
checksums leave live channels unchanged and do not refresh validity. A UART
framing error or a 500 us gap between bytes abandons an incomplete packet;
subsequent headers can restart reception. This implements the standard fourteen
16-bit channel layout in the project reference, not extended-channel variants
or iBUS sensor telemetry. Protocol reference: [Betaflight iBUS receiver source](https://github.com/betaflight/betaflight/blob/master/src/main/rx/ibus.c).

`rc_valid` indicates recent serial packets, not proof of a live RF connection.
FlySky supports configured channel values on RF loss; see its
[failsafe explanation](https://www.flysky-cn.com/journal/2021/2/21/about-the-flysky-remote-control-failsafe-function).
If the receiver continues transmitting valid packets with failsafe or held values,
the packet timeout cannot identify RF loss. Configure transmitter failsafe values
with throttle low and arm off, and verify receiver behavior. The flight computer
must implement disarming and rearming policy; this module supplies requests only.

## Basys3 wiring

Use the receiver's servo iBUS output in iBUS mode. Connect its signal to Pmod JA1
and share ground. Power the receiver from the regulated 5 V supply described in
the root reference, and verify/translate its signal to 3.3 V before the FPGA input.

These constraints can be merged into the flight-computer XDC, with matching
top-level port names. The pin assignments come from
[Digilent's Basys3 master XDC](https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc).
Only add the clock constraint once if your project already defines it.

```tcl
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -period 10.000 -name sys_clk [get_ports clk]
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports ibus_rx]
```

Constrain the asynchronous input path only to the first UART synchronizer stage
when integrating timing constraints; retain normal timing between the two stages.
The two synchronizer registers carry the Vivado `ASYNC_REG` attribute.

## Simulation

Open `tests/IBusReceiver.v` and use the existing VS Code Verilog runner, or run from
the `tests` directory:

```powershell
iverilog -Wall -s IBusReceiver_tb -o IBusReceiver.v.out IBusReceiver.v
vvp IBusReceiver.v.out
```

The test uses a real 100 MHz simulated clock, independently timed 115200-baud
serial bits, and a shortened 5 ms validity timeout. It checks all fourteen raw
channels, atomic updates, control mapping and remapping, clamping, corrupt checksums,
partial packets, glitches, bad stop bits, a stuck-low line, reset, and about 2%
transmitter baud variation. It writes `tests/IBusReceiver.vcd` with serial and
control signals. Use `vvp IBusReceiver.v.out +full_wave` to include the clock and
internal counters; the resulting waveform file is much larger.

Simulation verifies RTL behavior. Vivado synthesis, timing closure, and physical
receiver/transmitter behavior still need verification in the integrated design.
