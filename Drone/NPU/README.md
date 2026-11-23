# Lidar classifier integration

The supplied model is a single five-neuron linear classifier over 64 raw,
unsigned millimetre distances in sensor row order. Each score is a signed
32-bit dot product of distance and signed 16-bit weight, plus signed 32-bit
bias. Argmax chooses the lowest index on ties. The trained files are
`weights.mem` (320 neuron-major entries) and `biases.mem` (five entries).
No rescaling, mirroring, normalization or replacement of invalid sensor values
has been added to the supplied inference calculation.

Confirmed trained class order:

| Model index | Meaning | SW9 display |
| --- | --- | --- |
| 0 | Continue | Cont |
| 1 | Left | LEFt |
| 2 | Right | rIGt |
| 3 | Up | uP |
| 4 | Stop | StoP |

These indices differ from the flight controller's action enumeration. The board
test translates them for display using `NPU_CLASS_ACTIONS`; they are never sent
to the flight controller. Its `npu_available` and `avoidance_enable` inputs
remain tied low, including when the transmitter requests AI control.

`NPU.v` is the integration boundary. `ENABLE=0` retains the unavailable default
for the production `Drone` top. The battery/lights flight test explicitly enables
inference and feeds the coherent 64-pixel stream captured by `DepthFrameStream`.
The NPU stores a private snapshot before starting the supplied `layer` datapath.
Partial or out-of-order images do not start inference. The latest complete
prediction is held between frames; loss of lidar health clears availability
and aborts pending inference. A new complete frame is required after recovery.

SW9 shows the prediction instead of center distance. The existing configuration,
error and ultrasonic view priorities still apply. Position views have priority
only when ENABLE_POSITION_DIAGNOSTICS=1; the current hover image disables that
diagnostic estimator to fit FPGA resources. With no valid
prediction, SW9 shows dashes. LD9 continues to indicate lidar scan freshness,
not a classification confidence score. The 64-pixel USB packet is unchanged.

SystemVerilog support is required for the supplied array ports. `NPU.v` includes
the guarded helper files; add its directory to include paths. Vivado must treat
`NPU.v` as SystemVerilog and include both memory files as design sources. The
board build and test scripts do this. Memory-file parameters allow independent
simulation directories without changing the trained files.

`tests/NPU/generate_vectors.py` regenerates independent integer reference scores
from the model. Twenty frames cover every class, random scenes, all-zero input,
maximum unsigned input and a ramp. Tests compare all five scores as well as
argmax, and cover repeated inference, incomplete frames, health loss and reset.
Board integration tests check UART-to-inference routing, all five display
mappings, stale suppression, USB data preservation and motor-override isolation.
These tests establish digital arithmetic and wiring, not model accuracy in flight.

`lidar_npu_top.sv` is the supplied standalone reference top. The board uses its
existing lidar driver and the snapshot boundary instead, avoiding a second
sensor interface or contention with the USB viewer.
