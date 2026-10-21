# SEN0628 lidar array

`LidarArray` configures the DFRobot SEN0628 for 8x8 ranging, requests scans over
115200-baud 8N1 UART, and stores raw 16-bit millimeter distances in an image buffer.
`PeripheralInterface` instantiates it and `Drone` exposes its ports. All logic uses
`clk`; assert synchronous, active-high `rst` before use.

The interface shares `Drone/PeripheralInterface/UartReceiver.v` with the radio
and `Drone/UartTransmitter.v` with system UART output. Include all design `.v` files under `Drone/`
recursively. The radio's `BAUD_RATE` parameter does not change the lidar baud.

## Connections

| SEN0628 connection | FPGA connection |
| --- | --- |
| T (sensor TX) | `lidar_rx` |
| R (sensor RX) | `lidar_tx` |
| GND | Common ground |
| Power | Supply appropriate for the sensor board |

Select UART on the sensor and power-cycle after changing its mode switch, as
described in the [manufacturer's UART example](https://wiki.dfrobot.com/sen0628/docs/21559).
Both serial wires are required. The USB text output is a separate interface;
this module uses binary commands on the Gravity connector.

Assign `lidar_rx` and `lidar_tx` to two free board pins in your XDC. Basys3 I/O
must use 3.3 V logic; verify the sensor connector's signal levels with the chosen
supply and translate levels if necessary. No physical pins are selected here.
As with the radio, the development top exposes internal buses; a board wrapper
should connect the image read port to internal processing logic.

## Reading the depth image

| `Drone` / `PeripheralInterface` port | Meaning |
| --- | --- |
| `lidar_read_address[5:0]` | Input: `row * 8 + column`, equivalently `{row[2:0], column[2:0]}` |
| `lidar_distance_mm[15:0]` | Asynchronous read of the selected pixel in the published image |
| `lidar_image_valid` | A complete image has been received since reset |
| `lidar_frame_valid` | One `clk` pulse when a complete scan is published |
| `lidar_frame_count[31:0]` | Number of published scans since reset; wraps modulo 2^32 |
| `lidar_initialized` | A successful 8x8 configuration reply has been received; also high during settling |
| `lidar_error` | One `clk` pulse for a rejected response, framing error, or timeout |

The two banks each hold 64 pixels (128 bytes); total image storage is 256 bytes.
Incoming data writes the unpublished bank. Only the final byte of a complete,
accepted response switches the bank visible to readers. Partial responses never
change the published image. Reset clears validity and the frame counter, and
reads return zero until the first image is published; RAM itself needs no reset.

For example, set `lidar_read_address = {3'd2, 3'd5}` to read row 2, column 5
(address 21). Set an address before a rising clock edge and capture
`lidar_distance_mm` at that edge. After observing `lidar_frame_valid`, a reader
can walk addresses 0 through 63 over consecutive clocks. The buffer has no
reader lock or backpressure: a long read can straddle publications. Compare
`lidar_frame_count` before and after a full read and retry if it changed.

`lidar_image_valid` means stored data exists, not that it is recent. An error
retains the last image and its counter; consumers needing freshness must time
`lidar_frame_valid` or changes in `lidar_frame_count`. The NPU remains a placeholder
and does not consume this image yet. The flight-controller prototype accepts
the future NPU's action codes rather than reading depth pixels directly.

Pixels stay in the sensor's transmitted row order; no rotation, mirroring,
clipping, or grayscale conversion is applied. Check physical orientation using a
nearby target before assigning directions. Firmware V1.3 reports invalid points
as 4000; the buffer preserves that raw value, so it is not a separate validity
flag. See the [firmware notes](https://wiki.dfrobot.com/sen0628/docs/21564).

## UART protocol and timing

Protocol reference: [DFRobot_MatrixLidar.cpp](https://github.com/DFRobot/DFRobot_MatrixLidar/blob/master/DFRobot_MatrixLidar.cpp)
and its [matrix enumeration](https://github.com/DFRobot/DFRobot_MatrixLidar/blob/master/DFRobot_MatrixLidar.h).

| Transaction | Bytes in hexadecimal |
| --- | --- |
| Select 8x8 mode | `55 00 05 01 00 00 00 08` |
| Successful mode reply | `53 01 len_lo len_hi [payload]` |
| Request all pixels | `55 00 01 02` |
| Successful scan reply | `53 02 80 00`, then 128 payload bytes |

Request lengths are big endian and include the command. Reply lengths are little
endian and count only payload bytes. Each pixel is low byte then high byte.
Successful mode replies may have zero or up to 999 payload bytes, which are
drained. Scan replies must contain exactly 128 payload bytes. A `63` failure
status, unexpected status/command/length, UART framing error, or timeout abandons
the transaction. After a quiet byte-timeout interval, the module configures again.

This protocol has no checksum or transaction identifier. Validation cannot detect
every bit error or distinguish every very late reply from a new transaction.
Idle draining and one outstanding command prevent ordinary response overlap.

| Parameter on `Drone` / `PeripheralInterface` | Default | Purpose |
| --- | --- | --- |
| `CLK_FREQ` | 100,000,000 | System clock in Hz |
| `LIDAR_STARTUP_MS` | 1000 | Initial wait after reset |
| `LIDAR_SETTLE_MS` | 5000 | Wait after successful configuration, matching the reference library |
| `LIDAR_POLL_MS` | 100 | Delay from a completed scan to the next request |
| `LIDAR_RESPONSE_TIMEOUT_MS` | 8000 | Total reply deadline after the final request byte |
| `LIDAR_BYTE_TIMEOUT_MS` | 500 | Maximum gap after the first reply byte; also recovery quiet time |

On `LidarArray` itself, remove the `LIDAR_` prefix from these delay parameters.
All delays must be positive and fit a 32-bit clock counter. UART request bytes
have an additional 1 ms idle gap, following the reference library's pacing.
Actual frame intervals include request, reply, and sensor processing time;
`LIDAR_POLL_MS` is not a guaranteed frame rate. Defaults favor sensor startup and
recovery compatibility; use shorter overrides only after checking hardware.

## Validation

Run `./Run-Tests.ps1` or select `-Module LidarArray` / `-Module UartTransmitter`.
The lidar bench drives independently timed serial data and checks command bytes,
all 64 addresses, byte order, consecutive scans, atomic bank publication,
truncated replies, wrong lengths and commands, failed status, framing errors,
missing responses, recovery, and reset. It also exercises the full `Drone` /
`PeripheralInterface` path with non-default timing parameters. The transmitter
bench checks every byte, exact bit durations, busy handling, and reset.

These are simulations against the documented protocol, not captured sensor
traffic. Vivado synthesis/timing and operation with a physical SEN0628 still need
board verification.
