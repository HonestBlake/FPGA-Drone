# Accelerometer impacts in the flight test

ALLOW_ACCEL_CLIPPING defaults to 0 in shared IMU/ImuEstimator for compatibility.
The current BatteryLightsFlightTest explicitly enables it. At a saturated
accelerometer sample, accel_clipped is asserted and gravity correction is
skipped, but calibrated gyro attitude prediction continues. Calibration cannot
complete on clipped samples. Gyro clipping, stale data, sensor failures, rate
numeric overflow and the +/-35deg estimator envelope retain their fault behavior.
No sensor ranges, SPI setup, motor arming interlocks or gyro bias calibration
were bypassed. Accel clipping alone does not clear calibration or arm permission.
The diagnostic position tracker invalidates itself on clipping because lost
acceleration cannot be reconstructed; this does not affect motor commands.
