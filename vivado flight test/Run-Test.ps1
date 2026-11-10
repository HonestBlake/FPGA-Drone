$ErrorActionPreference = 'Stop'
$sharedRoot = Join-Path (Split-Path $PSScriptRoot -Parent) 'Drone'
$relativeSources = @(
    'PeripheralInterface/UartReceiver.v',
    'PeripheralInterface/RadioReceiver/RadioReceiver.v',
    'PeripheralInterface/RadioReceiver/IBusDecoder.v',
    'PeripheralInterface/RadioReceiver/RCMapper.v',
    'PeripheralInterface/IMU/imu_spi.v', 'PeripheralInterface/IMU/IMU.v',
    'PeripheralInterface/IMU/ImuEstimator.v',
    'PeripheralInterface/MotorControl/MotorControl.v',
    'PeripheralInterface/MotorControl/MotorDriver.v',
    'FlightController/FlightController.v', 'System/TickGenerator.v',
    'System/SevenSegmentDisplay.v'
)
$sources = @($relativeSources | ForEach-Object { Join-Path $sharedRoot $_ })
$sources += Join-Path $PSScriptRoot 'src/FlightTest.v'
$compiler = (Get-Command iverilog -ErrorAction Stop).Source
$simulator = (Get-Command vvp -ErrorAction Stop).Source
& $compiler -g2005 -Wall -s FlightTest -t null @sources
if ($LASTEXITCODE -ne 0) { throw 'Default board top compilation failed' }
Push-Location -LiteralPath (Join-Path $PSScriptRoot 'sim')
try {
    foreach ($benchName in @('FlightTest_tb','FlightClock_tb')) {
        & $compiler -g2005 -Wall -s $benchName -o "$benchName.v.out" @sources "$benchName.v"
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $benchName" }
        & $simulator "$benchName.v.out"
        if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $benchName" }
    }
} finally { Pop-Location }
