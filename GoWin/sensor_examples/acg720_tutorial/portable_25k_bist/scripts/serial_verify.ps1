#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Port = 'COM5',
    [int]$BaudRate = 115200,
    [int]$TimeoutMs = 6000,
    [int]$MinimumMatches = 2,
    [int]$SettleMs = 500,
    [string]$OutputLog = (Join-Path (Split-Path -Parent $PSScriptRoot) 'hardware_serial.log')
)

$ErrorActionPreference = 'Stop'
$expected = 'ACG720_TUTORIAL_25K B=1 P=1 G=1 C=1 D=1 U=1 ALL=1 S=B1E6'
$serial = [System.IO.Ports.SerialPort]::new(
    $Port, $BaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One
)
$serial.ReadTimeout = 100
$serial.WriteTimeout = 100
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$received = ''
$watch = [Diagnostics.Stopwatch]::StartNew()
try {
    $serial.Open()
    # Drain bytes buffered by the FTDI while the COM handle was closed, then
    # start a fresh timing window for repeated-line verification.
    Start-Sleep -Milliseconds $SettleMs
    $serial.DiscardInBuffer()
    $watch.Restart()
    while ($watch.ElapsedMilliseconds -lt $TimeoutMs) {
        $chunk = $serial.ReadExisting()
        if ($chunk) {
            $received += $chunk
            if ([regex]::Matches($received, [regex]::Escape($expected)).Count -ge $MinimumMatches) { break }
        }
        Start-Sleep -Milliseconds 20
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
    $watch.Stop()
}

$matchCount = [regex]::Matches($received, [regex]::Escape($expected)).Count
$result = if ($matchCount -ge $MinimumMatches) { 'PASS' } else { 'FAIL' }
@(
    "captured_at=$((Get-Date).ToString('o'))"
    "port=$Port"
    "baud=$BaudRate"
    "elapsed_ms=$($watch.ElapsedMilliseconds)"
    "matches=$matchCount"
    "result=$result"
    '---BEGIN_SERIAL---'
    $received
    '---END_SERIAL---'
) | Set-Content -LiteralPath $OutputLog -Encoding utf8

if ($matchCount -lt $MinimumMatches) {
    Write-Error "未在 $TimeoutMs ms 内收到至少 $MinimumMatches 条预期 25K BIST 行（实际 $matchCount 条）；原始数据已保存到 $OutputLog"
    exit 1
}
Write-Host "SERIAL_25K_BIST_PASS port=$Port baud=$BaudRate matches=$matchCount elapsed_ms=$($watch.ElapsedMilliseconds)"
Write-Host $expected
