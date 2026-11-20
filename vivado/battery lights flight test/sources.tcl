set test_dir [file dirname [file normalize [info script]]]
set shared_dir [file normalize [file join $test_dir .. .. Drone]]
set design_sources [list [file join $test_dir src BatteryLightsFlightTest.v]]
lappend design_sources [file join $test_dir src DepthFrameStream.v]
foreach rel {
    UartTransmitter.v
    System/UartOutput.v
    PeripheralInterface/LidarArray/LidarArray.v
    PeripheralInterface/Ultrasonic/Ultrasonic.v
    PeripheralInterface/UartReceiver.v
    PeripheralInterface/RadioReceiver/RadioReceiver.v
    PeripheralInterface/RadioReceiver/IBusDecoder.v
    PeripheralInterface/RadioReceiver/RCMapper.v
    PeripheralInterface/IMU/imu_spi.v
    PeripheralInterface/IMU/IMU.v
    PeripheralInterface/IMU/ImuEstimator.v
    PeripheralInterface/MotorControl/MotorControl.v
    PeripheralInterface/MotorControl/MotorDriver.v
    FlightController/FlightController.v
    FlightController/PositionEstimator.v
    System/TickGenerator.v
    System/BatteryMonitor.v
    System/SevenSegmentDisplay.v
} { lappend design_sources [file join $shared_dir $rel] }
