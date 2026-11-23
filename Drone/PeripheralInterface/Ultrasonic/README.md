# HC-SR04 ultrasonic range interface

`Ultrasonic.v` is a standalone, synthesizable pulse-width interface for the
HC-SR04. The current board test instantiates it with trigger on JC3 and divided
echo on JC4. SW10 shows millimetres, LD10 indicates sensor health, and valid
samples feed the [height controller](../../FlightController/HOVER.md).
The production `PeripheralInterface` / `Drone` top does not automatically enable
this bench integration.

All logic runs in `clk`; `rst` is synchronous active high. `enable=0` clears
measurement state, drives trigger low, and makes `valid=0`. Echo has a two-stage
synchronizer. All other inputs must be synchronous to this clock.

## Signals

| Port | Meaning |
| --- | --- |
| `enable` | Permit periodic ranging; tie low until intentionally integrated |
| `echo` | Level-shifted echo input, never raw 5V |
| `trigger` | Active-high trigger pulse, default 10 microseconds |
| `busy` | Trigger or measurement in progress |
| `distance_mm[15:0]` | Last good slant-range measurement; invalid results do not overwrite it |
| `sample_valid` | One clock pulse when a good distance is published |
| `measurement_done` | One clock pulse on either a good measurement or a failure |
| `valid` | Last completed measurement was good and ranging remains enabled |
| `fault_code[2:0]` | Persistent last outcome: 0 good, 1 missing rise, 2 echo stuck high, 3 outside range, 4 echo already high before trigger |

Default measurements start 65ms apart. Echo rise/fall must complete within a
35ms timeout after trigger. Failed attempts invalidate the reading and the next
period retries automatically. There are no overlapping triggers. The conversion
uses a fixed 340m/s speed of sound, rounded to the nearest millimeter; default
accepted range is 20..4000mm. Resolution is not accuracy: module variation,
temperature, target material, vibration, propeller noise and tilt affect results.
There is no temperature, tilt, median, or outlier compensation in this driver.

These timings/range follow the [HC-SR04 module sheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf),
which specifies a trigger of at least 10 microseconds and recommends a cycle
longer than 60ms. A low-clock simulation and full hardware clocks use the same
clock-derived counters. Consumers should also time out absent `sample_valid`
events; the paired HoverController has its own freshness watchdog.

## Planned wiring, not yet part of a bitstream

Root [pins.md](../../../pins.md) lists **JC1 trigger / JC2 echo**. JC3 remains
the receiver input in the battery/lights test. HC-SR04 VCC receives regulated
5V and GND shares the Basys3 ground. Reduce Echo with a divider, for example:

```text
HC-SR04 Echo -- 2.2k --+-- Basys3 JC2
                      |
                     3.3k
                      |
                     GND
```

This gives approximately 3.0V from a 5V Echo signal. The FPGA trigger is 3.3V;
verify that the particular HC-SR04 accepts this high level, otherwise use a
3.3V-to-5V buffer on trigger. Never connect a 5V signal directly to FPGA GPIO.
Provide normal FPGA input timing/synchronizer constraints when integrating.

## Verification

From the project root, run `./Run-Tests.ps1 -Module Ultrasonic`.
The bench checks exact trigger width/spacing, 20mm/4m boundaries, short and
overrange pulses, no echo, stuck echo, high before trigger, subsequent recovery,
disabled behavior, and reset. This is a digital echo model, not acoustic or
flight validation.
