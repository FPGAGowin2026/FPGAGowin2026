#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Port = 'COM5',
    [int]$BaudRate = 115200,
    [int]$TimeoutMs = 3000,
    [string]$LogPath = ''
)

$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
if (-not $LogPath) {
    $LogPath = Join-Path $projectDir 'reports\serial_debug.log'
}

function Invoke-FunctionalBistCommand {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [string]$Command,
        [string]$Expected,
        [int]$CommandTimeoutMs
    )
    $SerialPort.DiscardInBuffer()
    $SerialPort.Write($Command)
    $deadline = [DateTime]::UtcNow.AddMilliseconds($CommandTimeoutMs)
    $received = ''
    while ([DateTime]::UtcNow -lt $deadline) {
        $received += $SerialPort.ReadExisting()
        if ($received.Contains($Expected)) {
            return "PASS '$Command' -> $Expected"
        }
        Start-Sleep -Milliseconds 20
    }
    throw "命令 '$Command' 未在 ${CommandTimeoutMs}ms 内收到 '$Expected'。实际收到：$received"
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port, $BaudRate, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$serial.Encoding = [Text.Encoding]::ASCII
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$results = [System.Collections.Generic.List[string]]::new()
try {
    $serial.Open()
    Start-Sleep -Milliseconds 250
    $startup = $serial.ReadExisting()
    if ($startup) {
        $results.Add("STARTUP=$($startup.Replace("`r", '<CR>').Replace("`n", '<LF>'))")
    }
    $results.Add((Invoke-FunctionalBistCommand $serial 'P' "PONG;FUNC_BIST=READY`r`n" $TimeoutMs))
    $results.Add((Invoke-FunctionalBistCommand $serial 'B' "FUNC_BIST=PASS;MODULES=17/17;MODE=ONCHIP_STIMULUS`r`n" $TimeoutMs))
    $results.Add((Invoke-FunctionalBistCommand $serial 'D' "PASS=0x1FFFF;OBS=0x1FFFF;ACT=0x1FFFF;DONE=1`r`n" $TimeoutMs))
    $results.Add("SERIAL FUNCTIONAL BIST PASS: $Port @ $BaudRate 8-N-1")
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

$logDir = Split-Path -Parent $LogPath
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$results | Set-Content -LiteralPath $LogPath -Encoding utf8
$results | ForEach-Object { Write-Host $_ -ForegroundColor Green }
Write-Host "证据日志：$LogPath" -ForegroundColor Cyan
