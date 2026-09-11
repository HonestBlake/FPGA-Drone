# FlySky FS-iA6B iBUS Interface for Basys3

## Physical Interface

The FlySky FS-iA6B receiver supports traditional per-channel PWM outputs, but for this project the preferred interface is the dedicated **iBUS output**.

The FPGA only needs:

```text
VCC
GND
iBUS signal
```

The receiver can be powered from the regulated 5 V rail.

The Basys3 FPGA GPIO uses **3.3 V logic**, so the iBUS signal level must be verified before connecting it directly.

If the receiver outputs approximately 3.3 V logic:

```text
FS-iA6B iBUS
    |
    +--------> Basys3 Pmod GPIO
```

If it outputs approximately 5 V logic, use a level shifter or voltage divider.

Example divider:

```text
iBUS
 |
1 kΩ
 |
 +------> FPGA input
 |
2 kΩ
 |
GND
```

This reduces approximately 5 V to approximately 3.3 V.

All grounds must be common:

```text
Battery
  |
  +--> 5 V regulator
         |
         +--> Basys3
         +--> Receiver
         +--> Sensors

All grounds connected together
```

---

# Basys3 Pin Usage

The iBUS signal can connect to essentially any normal Basys3 Pmod GPIO pin.

For example:

```text
Pmod JA pin 1 -> ibus_rx
```

The top-level HDL interface might contain:

```verilog
input wire ibus_rx
```

The corresponding XDC entry would map that port to the physical Basys3 pin:

```tcl
set_property PACKAGE_PIN <JA1_PACKAGE_PIN> [get_ports ibus_rx]
set_property IOSTANDARD LVCMOS33 [get_ports ibus_rx]
```

Use the official Basys3 master XDC file for the exact package pin.

No special hardware UART pin is required. The UART receiver is implemented inside the FPGA.

---

# FPGA Input Synchronization

The iBUS signal is asynchronous relative to the Basys3 100 MHz clock.

The raw signal should therefore pass through a small synchronizer before being used by the UART receiver.

Conceptually:

```text
iBUS pin
   |
   v
flip-flop
   |
   v
flip-flop
   |
   v
UART receiver
```

Example:

```verilog
reg rx_meta;
reg rx_sync;

always @(posedge clk) begin
    rx_meta <= ibus_rx;
    rx_sync <= rx_meta;
end
```

The UART receiver then uses `rx_sync`.

---

# iBUS Serial Configuration

iBUS uses a UART-style serial interface.

Typical settings are:

```text
Baud rate: 115200
Data bits: 8
Parity:    none
Stop bits: 1

Format: 8N1
```

The Basys3 runs at:

```text
100 MHz
```

Therefore:

```text
100,000,000 / 115,200 ≈ 868 clock cycles per UART bit
```

One iBUS bit lasts approximately:

```text
8.68 µs
```

---

# UART Receiver Module

The first logical module should be a generic UART receiver.

It should not know anything about FlySky or iBUS.

Its job is simply:

```text
serial waveform
      |
      v
8-bit byte
```

A reasonable interface would be:

```verilog
module uart_rx (
    input  wire      clk,
    input  wire      rst,
    input  wire      rx,

    output reg [7:0] data,
    output reg       data_valid
);
```

The UART receiver should:

```text
1. Wait for the falling edge of the start bit
2. Wait approximately half a bit period
3. Verify the start bit is still low
4. Wait one full bit period
5. Sample bit 0
6. Repeat for bits 1-7
7. Check the stop bit
8. Output the received byte
9. Pulse data_valid
```

Output behavior:

```text
data       = received byte
data_valid = high for one FPGA clock cycle
```

---

# iBUS Frame Format

The iBUS decoder consumes bytes from the UART receiver.

A normal iBUS RC frame contains 32 bytes.

Typical structure:

```text
Byte 0      0x20
Byte 1      0x40

Bytes 2-3   Channel 1
Bytes 4-5   Channel 2
Bytes 6-7   Channel 3
Bytes 8-9   Channel 4
...
Bytes 28-29 Channel 14

Bytes 30-31 Checksum
```

Each channel value is a 16-bit little-endian integer.

For example:

```text
low byte  = 0xDC
high byte = 0x05

value = 0x05DC
      = 1500
```

Typical RC channel values are approximately:

```text
1000 = minimum
1500 = center
2000 = maximum
```

---

# iBUS Decoder

The decoder should be separate from the UART receiver.

Recommended structure:

```text
ibus_rx
   |
   v
uart_rx
   |
   | byte
   | byte_valid
   v
ibus_decoder
```

A simple decoder state machine could be:

```text
WAIT_HEADER_1
      |
      | receive 0x20
      v
WAIT_HEADER_2
      |
      | receive 0x40
      v
READ_CHANNEL_DATA
      |
      v
READ_CHECKSUM
      |
      v
VALIDATE_FRAME
```

If a valid frame is received:

```text
update channel outputs
```

If the checksum fails:

```text
discard frame
return to WAIT_HEADER_1
```

---

# Temporary vs Live Channel Registers

The flight controller should never see a partially received frame.

Do not update channel outputs as each byte arrives.

Instead:

```text
incoming bytes
      |
      v
temporary registers
      |
      v
checksum valid?
   /       \
 no         yes
 |           |
discard      copy into live channel registers
```

This ensures that all channel values change together only after a complete valid frame arrives.

---

# Channel Outputs

The iBUS decoder should remain generic.

It should output things such as:

```text
ch1
ch2
ch3
ch4
ch5
ch6
frame_valid
rc_valid
```

Do not hard-code meanings such as `pitch` or `throttle` inside the decoder.

The actual transmitter channel mapping should be verified experimentally.

A likely mapping is:

```text
CH1 -> roll
CH2 -> pitch
CH3 -> throttle
CH4 -> yaw
CH5 -> arm switch
CH6 -> flight mode / obstacle avoidance enable
```

But verify this by moving each stick and switch individually and observing which channel changes.

---

# RC Mapper

A separate `rc_mapper` module should convert raw iBUS channel values into values useful to the flight controller.

For centered controls:

```text
iBUS value      Internal command

1000            -500
1500               0
2000            +500
```

This could be used for:

```text
roll
pitch
yaw
```

Throttle should instead map approximately:

```text
1000 -> 0
2000 -> 1000
```

The mapper might therefore output:

```text
throttle        0 ... 1000

roll_command   -500 ... +500
pitch_command  -500 ... +500
yaw_command    -500 ... +500

arm_request
avoidance_enable
rc_valid
```

---

# Connection to the Flight Computer

The full data path should be:

```text
FlySky transmitter
        |
        | RF
        v
FS-iA6B receiver
        |
        | iBUS
        v
Basys3 GPIO
        |
        v
Input synchronizer
        |
        v
UART receiver
        |
        v
iBUS decoder
        |
        v
RC mapper
        |
        +--> throttle
        +--> roll command
        +--> pitch command
        +--> yaw command
        +--> arm request
        +--> mode
        +--> rc_valid
        |
        v
Flight controller
        |
        v
PID / stabilization
        |
        v
Motor mixer
        |
        v
ESC output
```

The flight controller should not know anything about raw iBUS packets.

Its input should only be clean control values.

---

# Receiver Timeout / Failsafe

The radio subsystem should also detect loss of communication.

Every valid iBUS frame should:

```text
reset timeout counter
rc_valid = 1
```

If no valid frame is received for some period, for example:

```text
50-100 ms
```

then:

```text
rc_valid = 0
```

The flight controller can then enter failsafe.

For example:

```text
if rc_valid == 0:
    disarm motors
```

The flight controller should never continue indefinitely using the last received stick values.

---

# Recommended Module Structure

```text
radio/
├── uart_rx.sv
├── ibus_decoder.sv
└── rc_mapper.sv
```

The logical boundary between the radio subsystem and the rest of the flight computer should look approximately like:

```text
Input:
    ibus_rx

Outputs:
    throttle
    roll_command
    pitch_command
    yaw_command
    arm_request
    avoidance_enable
    rc_valid
```

This keeps the design modular:

```text
UART layer
    handles serial bits

iBUS layer
    handles packets and channels

RC mapper
    handles pilot-command interpretation

Flight controller
    handles stabilization and motor control
```
