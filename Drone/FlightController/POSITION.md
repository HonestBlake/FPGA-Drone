# Experimental IMU-only position diagnostics

PositionEstimator uses native FLU accelerometer counts (16384/g), controller
roll/pitch (right-down/nose-up centidegrees), and positive clockwise yaw rate.
It rotates specific force into a local FLU frame using small-angle roll/pitch and a 128-bin yaw lookup,
subtracts gravity, calibrates constant stationary acceleration bias while idle,
and integrates velocity and position at a nominal 500Hz using a rounded Q16 time step (about 0.05% time-step error). X is initially forward,
Y initially left, Z up; origin and gyro-relative yaw reset on tracking disable.
It has no absolute position or compass reference and must not close a motor loop.

The board enables tracking only while actually armed with throttle >=10. While
idle it collects 64 good stationary samples after the IMU becomes valid; allow
at least 0.13s for this additional diagnostic calibration. SW11 displays local X
and SW12 local Y in signed centimetres, using three digits plus sign (clamped
at 999cm). Both selected means X. These views follow configuration/error views
and precede other diagnostic views. LD11/12 remain radio AI/hover request LEDs.

The rotation model is restricted to +/-15 degrees roll/pitch and yaw lookup
resolution is 2.8125 degrees. These approximations add diagnostic error.
Data is invalidated outside this diagnostic angle envelope, on clipping, missing/unhealthy IMU, sample interval outside
0.5..1.5 nominal periods, 10 seconds of unaided tracking, or numeric bounds of
10m and 10m/s. Invalid data displays dashes; idle resets the tracking origin.
A valid flag means internally consistent recent data, not localization accuracy.
A missed acceleration sample loses information, so tracking stays degraded until
origin reset. Normal motor stabilization is independent of this diagnostic flag.

The IMU-only estimate accumulates position error from residual bias, attitude
error, vibration, clipping and sample timing. Even a 1 degree gravity-removal
error produces about 0.17m/s^2 horizontal acceleration error (roughly 8.5m false
position in 10s). It is not a GPS/optical-flow substitute. No pilot target or
position-hold motor feedback is implemented, per the diagnostic-only agreement.

Reference: https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-inserrorbudget

Tests cover stationary gravity removal, acceleration bias, constant acceleration,
rotation into the initial yaw frame, time/health/clipping invalidation and origin
reset. Hardware accuracy is not established by these simulations.

The board sets TRACK_Z=0 to save FPGA resources; its diagnostic views are X/Y only. The standalone module defaults to full X/Y/Z integration. Calibration storage is sized from the sample count.
