param(
    # Optional module name, e.g. -Module UartReceiver. Layout is always checked.
    [string]$Module = '*'
)

$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$sourceRoot = Join-Path $projectRoot 'Drone'
$testRoot = Join-Path $projectRoot 'tests'
$compiler = (Get-Command iverilog -ErrorAction Stop).Source
$simulator = (Get-Command vvp -ErrorAction Stop).Source
$sources = @(Get-ChildItem -LiteralPath $sourceRoot -Recurse -File -Filter '*.v' | Sort-Object FullName)
$benches = @(Get-ChildItem -LiteralPath $testRoot -Recurse -File -Filter '*_tb.v')
$expectedBenches = @()

# Each subsystem has a matching entry module; helper modules share its folder.
foreach ($directory in @((Get-Item -LiteralPath $sourceRoot)) + @(Get-ChildItem -LiteralPath $sourceRoot -Recurse -Directory)) {
    $entryModule = Join-Path $directory.FullName ($directory.Name + '.v')
    if (-not (Test-Path -LiteralPath $entryModule -PathType Leaf)) {
        throw "Expected subsystem entry module: $entryModule"
    }
}
foreach ($source in $sources) {
    $relativeSource = $source.FullName.Substring($sourceRoot.Length + 1)
    $testPath = Join-Path $testRoot ($relativeSource -replace '\.v$', '_tb.v')
    if (-not (Test-Path -LiteralPath $testPath -PathType Leaf)) {
        throw "Missing mirrored test: $testPath"
    }
    $declarations = [regex]::Matches((Get-Content -Raw -LiteralPath $source.FullName), '(?m)^module\s+(\w+)')
    if ($declarations.Count -ne 1 -or $declarations[0].Groups[1].Value -cne $source.BaseName) {
        throw "Expected a single module named $($source.BaseName) in $($source.FullName)"
    }
    $expectedBenches += $testPath
}
foreach ($bench in $benches) {
    if ($expectedBenches -notcontains $bench.FullName) { throw "Unmatched testbench: $($bench.FullName)" }
}

Write-Output "Layout OK: $($sources.Count) modules, $($benches.Count) mirrored testbenches."
# Compile the source hierarchy independently of testbench includes.
& $compiler -g2005 -Wall -s Drone -t null @($sources.FullName)
if ($LASTEXITCODE -ne 0) { throw 'Drone design compilation failed' }

$selected = @($benches | Where-Object { ($_.BaseName -replace '_tb$', '') -like $Module } | Sort-Object FullName)
if ($selected.Count -eq 0) { throw "No testbench matches module '$Module'" }
foreach ($bench in $selected) {
    Write-Output "Testing $($bench.FullName.Substring($projectRoot.Length + 1))"
    Push-Location -LiteralPath $bench.DirectoryName
    try {
        # Running in the test folder also matches the VS Code Verilog runner.
        $compiledName = $bench.BaseName + '.v.out'
        & $compiler -g2005 -Wall -s $bench.BaseName -o $compiledName $bench.Name
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($bench.Name)" }
        & $simulator $compiledName
        if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $($bench.Name)" }
        $waveform = Join-Path $bench.DirectoryName (($bench.BaseName -replace '_tb$', '') + '.vcd')
        if (-not (Test-Path -LiteralPath $waveform -PathType Leaf) -or (Get-Item -LiteralPath $waveform).Length -eq 0) {
            throw "Missing waveform: $waveform"
        }
    } finally {
        Pop-Location
    }
}
Write-Output "PASS: all $($selected.Count) selected module tests."
