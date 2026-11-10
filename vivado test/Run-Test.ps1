$ErrorActionPreference = 'Stop'
$compiler = (Get-Command iverilog -ErrorAction Stop).Source
$simulator = (Get-Command vvp -ErrorAction Stop).Source
$sources = @(Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'src') -Filter '*.v' -File)
$bench = Join-Path $PSScriptRoot 'sim/RadioDisplayTest_tb.v'
$compiled = Join-Path $PSScriptRoot 'sim/RadioDisplayTest_tb.v.out'
# Check the top at actual board defaults, then simulate with faster parameters.
& $compiler -g2005 -Wall -s RadioDisplayTest -t null @($sources.FullName)
if ($LASTEXITCODE -ne 0) { throw 'Board top compilation failed' }
& $compiler -g2005 -Wall -s RadioDisplayTest_tb -o $compiled @($sources.FullName) $bench
if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
& $simulator $compiled
if ($LASTEXITCODE -ne 0) { throw 'Simulation failed' }
