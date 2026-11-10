$ErrorActionPreference = 'Stop'
$compiler = (Get-Command iverilog -ErrorAction Stop).Source
$simulator = (Get-Command vvp -ErrorAction Stop).Source
$sources = @(Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'src') -Filter '*.v' -File)
$bench = Join-Path $PSScriptRoot 'sim/RadioMotorTest_tb.v'
$compiled = Join-Path $PSScriptRoot 'sim/RadioMotorTest_tb.v.out'
& $compiler -g2005 -Wall -s RadioMotorTest -t null @($sources.FullName)
if ($LASTEXITCODE -ne 0) { throw 'Board top compilation failed' }
& $compiler -g2005 -Wall -s RadioMotorTest_tb -o $compiled @($sources.FullName) $bench
if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
& $simulator $compiled
if ($LASTEXITCODE -ne 0) { throw 'Simulation failed' }
