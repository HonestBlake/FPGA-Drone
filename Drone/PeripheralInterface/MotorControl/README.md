# Four-motor ESC output

`MotorControl` drives four signal inputs on the AERO SELFIE 45A 4-in-1 ESC.
The manufacturer's [45A product page](https://aeroselfie.myshopify.com/products/45a-4-in-1-esc-brushless-motor-speed-controller)
lists DShot support. Defaults use ordinary, unidirectional DShot300 with a
1 kHz update rate and the Basys3's 100 MHz clock. Confirm the installed ESC
firmware accepts DShot300 when commissioning the board; the Amazon short link
could not be resolved during implementation.

`MotorControl.v` instantiates the reusable `MotorDriver.v` four times as
`motor_front_right`, `motor_front_left`, `motor_back_right`, and `motor_back_left`.
All four packets start together and latch separate throttle values. This module
generates ESC commands; the ESC handles the motor's three-phase power switching.

## Commands and timing

| Input | Meaning |
| --- | --- |
| `front_right_throttle`, `front_left_throttle`, `back_right_throttle`, `back_left_throttle` | Separate 11-bit DShot values: 0 stops, 48..2047 sets throttle |
| `motors_armed` | Permit nonzero throttle after startup |
| `command_valid` | Caller guarantees current, valid motor commands; drop on stale data |
| `rst` | Synchronous active-high reset; assert before use |

Values 1..47 are reserved ESC commands and are converted to stop. Telemetry is
disabled. Packets contain 11 throttle bits, a zero telemetry bit, and the
four-bit XOR checksum, sent most significant bit first. See the
[Betaflight DShot specification](https://betaflight.com/docs/development/API/Dshot).
The RC throttle range of 0..1000 is mapped by `FlightController` to collective
DShot thrust before stabilization corrections and mixing. It is not connected
directly to this module.

`MotorDriver` accepts a one-clock `send` pulse while `busy=0`, latches the packet,
and ignores `send` during transmission. A zero bit is high for 3/8 of its period;
a one bit is high for 3/4. Clock counts round to the nearest integer. At 100 MHz,
DShot300 uses 333 clocks/bit, 125 clocks high for zero, and 250 for one. Each
packet has 16 bits followed by a low gap of two bit periods. `busy` includes
that gap. `DSHOT_RATE` also supports 150000 and 600000; confirm ESC support before
selecting either. The clock must provide at least eight clocks per bit.

`UPDATE_HZ` must divide `CLK_FREQ` exactly, with its period longer than the
packet plus gap. A dedicated `TickGenerator` controls output cadence independently
of `Drone.control_tick`. New values are sampled together at each packet start;
the caller must publish all four throttles atomically in this clock domain.

For the first `STARTUP_MS` (default 1000), only stop packets are sent. The startup
duration rounds up to a whole number of update periods. `ready` means that local
startup interval has elapsed; it is neither ESC acknowledgement nor confirmation
that the motors are armed. Reset restarts the interval. If the ESC is powered or
restarted later than the FPGA, reset this module after ESC power-up so the stop
interval runs again. The startup interval is configurable and needs hardware
verification with the installed firmware.

Disarming or dropping `command_valid` changes the next packet to stop; the
current packet completes without corruption. With the defaults, a stop packet
starts within one millisecond and finishes about 53.3 microseconds later. This
interface has no internal command watchdog: the caller must deassert
`command_valid` on timeout. Reset forces every output low on the next clock.

## Integration and wiring

`Drone` exposes `esc_front_right`, `esc_front_left`, `esc_back_right`, and
`esc_back_left`, plus `motor_ready` and `motor_busy`. Its `MOTOR_DSHOT_RATE`,
`MOTOR_UPDATE_HZ`, and `MOTOR_STARTUP_MS` parameters forward through `PeripheralInterface` to `MotorControl`.
The instance lives at `Drone.peripheral_interface.motor_control`; source and
mirrored tests live under their respective `PeripheralInterface/MotorControl/`
folders.
`FlightController` generates four separate motor throttle outputs using its
prototype X-quad mixer. It requires a fresh healthy IMU, valid radio input,
`motor_ready`, control ticks, and an explicit idle arm sequence before arming.
Without the external IMU/fusion module connected, the integrated design sends
only stop packets. See the [flight-computer guide](../../FlightController/README.md)
for the assumed motor rotation pattern, arming sequence, and gain/feedback contract.
`PeripheralInterface` gates its `motor_command_valid` input with `rc_valid`;
`Drone` supplies `flight_command_valid` as `motor_command_valid`; this checks
radio, IMU, motor readiness, and control-loop freshness. The separate
`flight_controller_available` signal only reports that the implementation exists.

Connect each `esc_*` output to the ESC signal input for the motor actually mounted
at that position, and connect FPGA ground to ESC signal ground. Use the
manufacturer's [connection diagram](https://aeroselfie.myshopify.com/cdn/shop/files/45A_4in1_ESC_Connection_Diagram.jpg)
and the labels on the actual board to identify the connector. Position names do
not assume a particular ESC channel order or motor rotation direction. Assign
four available FPGA GPIO pins with `LVCMOS33` in your board wrapper's XDC.
No pins are assigned here because the project's motor connector has not been chosen.

The product text does not specify signal voltage thresholds; verify that this
board revision accepts 3.3 V logic before connecting Basys3 GPIO. Connect battery
power to the ESC's power terminals, and each motor's three leads to its ESC power
outputs. Keep battery voltage and the ESC current-sense output away from FPGA GPIO.
Perform initial motor checks with propellers removed. Firmware configuration for
direction, special DShot commands, and bidirectional RPM telemetry are outside
this module's interface.

## Verification

Run `./Run-Tests.ps1 -Module 'Motor*'` or the full suite. The motor benches decode
output waveforms and check known packets, DShot150/300/600 pulse timing, reserved
command rejection, input latching, busy-send rejection, the low gap, independent
motor values, startup stop frames, periodic updates, disarming, command validity,
and reset. Simulation does not establish electrical compatibility or motor spin.
