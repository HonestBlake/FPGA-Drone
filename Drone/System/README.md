# Basys3 system services

`System` provides battery monitoring, a beacon LED, the control tick, a queued UART output, and a driver for the
Basys3 four-digit seven-segment display. `Drone` exposes the same write interfaces
and physical outputs. Everything runs on `clk` (100 MHz by default), with a
synchronous active-high `rst` that must be asserted before use.

## Battery voltage and charge estimate

`BatteryMonitor` instantiates the Artix-7 XADC directly; no XADC Wizard IP is
required. `System` and `Drone` expose `battery_vin_p` and `battery_vin_n`.
Feed the externally scaled **0..1 V** signal into P and analog ground into N.
The transfer is **0..16 V pack = 0..1 V ADC**; 12.6 V produces 0.7875 V.
Do not connect the pack directly to FPGA pins. The default auxiliary channel is
VAUX6 (Basys3 JXADC XA1_P/J3 and XA1_N/K3).

| Output | Meaning |
| --- | --- |
| `battery_raw[11:0]` | Last 12-bit ADC code |
| `battery_mv[15:0]` | Pack millivolts, rounded from `raw * 16000 / 4096` |
| `battery_percent[6:0]` | Linear voltage estimate, rounded and clamped to 0..100 |
| `battery_valid` | At least one measurement received since reset |
| `battery_sample_valid` | One clock pulse when all battery outputs update |

Defaults match the OVONIC **3S 11.1 V 2200 mAh LiPo** recorded in
[PARTS.md](../../PARTS.md): `BATTERY_EMPTY_MV=9000` (3 x 3.0 V) and
`BATTERY_FULL_MV=12600` (3 x 4.2 V). The empty endpoint is the LiPo discharge
floor, **not a flight/landing threshold**. See the
[OVONIC/Ampow manual](https://www.ampow.com/pages/user-manual).
The formula is `100 * (battery_mv - EMPTY_MV) / (FULL_MV - EMPTY_MV)`.
For example, 10.8 V gives approximately 50%. Voltage alone is an approximate
charge indication: load sag, temperature, and cell imbalance affect it. Total
voltage cannot establish individual-cell health. The pack marked swollen in the
parts list remains retired. This monitor does not change arming or motor control.

Parameters `BATTERY_XADC_CHANNEL` (0..15, default 6), `BATTERY_EMPTY_MV`, and
`BATTERY_FULL_MV` are forwarded through both enclosing modules. Standalone
`BatteryMonitor` uses names `XADC_CHANNEL`, `EMPTY_MV`, and `FULL_MV`.
Use channel-compatible physical pins if changing the channel. Reset clears all
values and validity; otherwise outputs retain the last sample between updates.
Validity is a first-sample flag, not a sensor-disconnect detector.

The XADC runs continuously in unipolar single-channel mode with extended
acquisition, reads its result through DRP on EOC, and captures bits 15:4 on DRDY.
The clock divider keeps ADCCLK at or below 26 MHz (25 MHz with the default
100 MHz system clock). Configuration and transfer-function reference:
[AMD UG480](https://docs.amd.com/r/en-US/ug480_7Series_XADC/Control-Registers).
Divider impedance and filtering must meet the XADC's input settling requirements.
Only one XADC primitive can own the chip's ADC; share it if more channels are added.

## Beacon LED

`LedBlinker` drives active-high `beacon_led` through `System` and `Drone`.
Defaults are **60 flashes/minute**, with **100 ms on / 900 ms off**.
Reset turns it off; the first rising clock after reset starts a fresh pulse.
Set `LED_FLASHES_PER_MINUTE` and `LED_PULSE_MS` on either enclosing module;
standalone names are `FLASHES_PER_MINUTE` and `PULSE_MS`. Period and pulse length
round to whole clocks and require at least one on and one off clock. Use the
actual `CLK_FREQ` when integrating.

The default is inside the 40..100 flashes/minute anticollision range in
[FAA AC 43.13-2B, section 5](https://www.faa.gov/documentlibrary/media/advisory_circular/ac%2043.13-2b.pdf).
Flash timing alone does not establish FAA compliance, brightness, color, or
visibility coverage. An external high-power light needs a suitable driver.

`Run-Tests.ps1 -Module BatteryMonitor` sweeps all 4096 codes and custom endpoints.
`-Module LedBlinker` checks pulse/period timing and reset. The Icarus tests use
`tests/support/XadcModel.vh`, a digital DRP test double; do not add it to Vivado
design sources. Analog conversion accuracy and divider hardware need board tests.

The Basys3 includes an FT2232HQ USB-UART bridge on its PROG/J4 micro-USB connector.
The FPGA supplies the serial bits; USB conversion is handled by that bridge. Its
display is multiplexed, with active-low segment and digit-enable pins, so the FPGA
also supplies display scanning. See sections 5 and 8.1 of the
[Digilent reference manual](https://digilent.com/reference/_media/reference/programmable-logic/basys-3/basys3_rm.pdf).

## Sending UART messages

`UartOutput` accepts a stream of bytes into a FIFO and sends them as idle-high,
LSB-first 8N1 UART. The default baud rate is 115200. ASCII bytes produce readable
text in a serial terminal; binary bytes can carry other data. Message formatting
and any separators or line endings come from the producer.

| `System` / `Drone` port | Meaning |
| --- | --- |
| `uart_data[7:0]` | Byte supplied by FPGA logic |
| `uart_valid` | Producer has a byte available |
| `uart_ready` | Queue can accept a byte this clock |
| `uart_busy` | A byte is queued or being transmitted, including its stop bit |
| `uart_tx` | Physical serial output to the onboard USB bridge |

A byte transfers on each rising edge with `uart_valid && uart_ready`. Hold both
`uart_valid` and `uart_data` stable while `uart_ready` is low. Advance to the next
byte only after acceptance. Holding valid high after acceptance submits another
byte at each subsequent ready edge. The default queue holds 16 waiting bytes,
plus one byte in flight. `uart_ready` stays low during reset; reset discards the
queue and aborts any partial transmission, returning TX to idle high.

For example, this producer sends `OK\r\n` once after each reset. Its signals connect
to the corresponding `System` or `Drone` ports; `uart_data` is declared as a `reg`
in the producer and `uart_valid` as a wire.

```verilog
reg [2:0] message_index;
assign uart_valid = !rst && message_index < 4;
always @* begin
    case (message_index)
        0: uart_data = "O";
        1: uart_data = "K";
        2: uart_data = 8'h0d; // CR
        3: uart_data = 8'h0a; // LF
        default: uart_data = 0;
    endcase
end
always @(posedge clk) begin
    if (rst) message_index <= 0;
    else if (uart_valid && uart_ready)
        message_index <= message_index + 1'b1;
end
```

Connect a USB data cable to PROG/J4 and open the board's COM port at **115200,
8 data bits, no parity, 1 stop bit, no flow control**. `uart_tx` uses FPGA pin A18.
This is independent of the radio and lidar UARTs. No host receive interface is
implemented here.

`Drone/UartTransmitter.v` is the shared bit serializer used by both `UartOutput`
and `LidarArray`. It lives at their deepest common directory, `Drone/`. Update
any manually maintained Vivado source list to use this path.

## Writing the display

`SevenSegmentDisplay` latches a four-digit value and automatically refreshes it.
Inputs are accepted on any rising edge where `display_write` is high. Inputs can
change afterward; the display retains the last write. Multiple writes before a
sweep use the latest value, with no queue or acknowledgement required.

| `System` / `Drone` port | Meaning |
| --- | --- |
| `display_write` | Write strobe for the value, enables, and decimal points |
| `display_value[15:0]` | Four hexadecimal nibbles, most significant on the left |
| `display_digit_enable[3:0]` | Logical 1 enables a digit; bit 0 is rightmost |
| `display_decimal_points[3:0]` | Logical 1 lights the matching decimal point |
| `seg[6:0]` | Active-low physical segments; bit 0 is A, bit 6 is G |
| `dp` | Active-low physical decimal point |
| `an[3:0]` | Active-low digit enables; AN0 is rightmost |

Example write values (pulse `display_write` for one clock with them):

```verilog
display_value          = 16'h1234;
display_digit_enable   = 4'b1111;
display_decimal_points = 4'b0100; // Shows 12.34
```

Set `display_decimal_points=0` for no points. `display_digit_enable=4'b0011`
shows only the rightmost two digits. Writing an enable mask of zero blanks the
display. Reset blanks it and clears its stored values.

Digits support `0` through `9`, `A`, `b`, `C`, `d`, `E`, and `F`. For a decimal
number, supply packed BCD digits: `16'h1234` displays decimal 1234, whereas
`16'd1234` is hexadecimal `04D2` and displays `04d2`. There is no automatic
binary-to-decimal conversion or arbitrary text renderer.

The driver snapshots all stored digits at the beginning of a full sweep so a
write cannot mix old and new digits within that sweep. A write at the boundary is
included immediately; otherwise it appears at the next boundary. Defaults give
250 full sweeps/second, with a 1 ms slot per digit. Each slot begins with 10 us of
blanking: the previous digit is disabled before segment changes, and segments
settle before the next digit is enabled. Scan outputs are registered and use
clock enables, not another clock domain.

## Parameters and board integration

| Parameter on `System` / `Drone` | Default | Meaning |
| --- | --- | --- |
| `CLK_FREQ` | 100,000,000 | Clock frequency in Hz |
| `CONTROL_TICK_FREQ` | 1,000 | Existing control tick; must divide `CLK_FREQ` |
| `UART_BAUD_RATE` | 115,200 | Host UART baud; independent of radio `BAUD_RATE` |
| `UART_FIFO_DEPTH` | 16 | Number of waiting bytes; any positive depth |
| `DISPLAY_REFRESH_HZ` | 250 | Requested full sweeps/second |
| `DISPLAY_BLANK_CYCLES` | `max(2, CLK_FREQ/100000)` | Blank clocks per digit slot |

The UART rounds its bit period to the nearest whole system clock and requires at
least eight clocks per bit. Display slots use `CLK_FREQ / (4 * DISPLAY_REFRESH_HZ)`
with integer truncation; the result must exceed `DISPLAY_BLANK_CYCLES`, which
must be at least two. Module-local names are `BAUD_RATE`/`FIFO_DEPTH` for
`UartOutput` and `REFRESH_HZ`/`BLANK_CYCLES` for `SevenSegmentDisplay`.

[constraints/Basys3-System.xdc](../../constraints/Basys3-System.xdc)
maps `uart_tx`, `seg`, `dp`, and `an` using
[Digilent's master XDC](https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc).
Merge those assignments into a board wrapper's constraints, along with the
100 MHz clock, synchronized reset, and other device pins. These are partial
output constraints, not a complete bitstream-ready board project.

The write ports are FPGA-internal interfaces: connect them to producer logic in
the board wrapper. Tie unused `uart_valid` and `display_write` low and tie unused
data inputs to zero. The development `Drone` top still exposes many internal
signals for testing; they should not all be mapped to physical I/O.

## Tests

`./Run-Tests.ps1` runs the complete suite. Select `-Module UartOutput`,
`-Module UartTransmitter`, `-Module SevenSegmentDisplay`, or `-Module System`
for individual tests. Coverage includes every UART byte, ASCII messages,
backpressure, simultaneous queue read/write, non-power-of-two pointer wrap,
reset while busy, all hexadecimal glyphs, digit order, decimal points, blanking,
and coherent writes. `System_tb` and `Drone_tb` send a real UART message and write
the display through both enclosing interfaces with non-default parameters.

Simulation passes do not establish Vivado timing closure or physical operation;
the modules and pin assignments still need verification on the board.

### Filtered battery updates

The XADC now enables 256-conversion hardware averaging. BatteryMonitor also
averages FILTER_SAMPLES complete readings (power of two, default 64), then
publishes the latest averaged value at UPDATE_HZ (default 4Hz). Outputs hold
between publication ticks. First validity requires a completed average and a
publication tick. battery_percent_tenths reports 0..1000 (0.0..100.0%).
System/Drone forward BATTERY_FILTER_SAMPLES and BATTERY_UPDATE_HZ. Integer
battery_percent remains available for existing status consumers. This filtering
reduces display chatter; it does not correct divider or grounding errors.
