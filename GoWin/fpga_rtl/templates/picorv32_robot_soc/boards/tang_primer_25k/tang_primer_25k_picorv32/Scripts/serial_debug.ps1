#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Port = 'COM5',
    [int]$BaudRate = 115200,
    [int]$TimeoutMs = 2500
)

$ErrorActionPreference = 'Stop'

function Invoke-PicoCommand {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [string]$Command,
        [string]$ExpectedPattern,
        [int]$CommandTimeoutMs
    )

    $SerialPort.DiscardInBuffer()
    $SerialPort.Write($Command)
    $deadline = [DateTime]::UtcNow.AddMilliseconds($CommandTimeoutMs)
    $received = ''
    while ([DateTime]::UtcNow -lt $deadline) {
        $received += $SerialPort.ReadExisting()
        if ($received -match $ExpectedPattern) {
            Write-Host "PASS '$Command' -> $($received.Trim())" -ForegroundColor Green
            return $received
        }
        Start-Sleep -Milliseconds 20
    }
    throw "命令 '$Command' 未在 ${CommandTimeoutMs}ms 内匹配 /$ExpectedPattern/。实际收到：$received"
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.Encoding = [Text.Encoding]::ASCII
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false

try {
    $serial.Open()
    Start-Sleep -Milliseconds 150
    $startup = $serial.ReadExisting()
    if ($startup) {
        Write-Host "启动数据：$($startup.Replace("`r", '<CR>').Replace("`n", '<LF>'))"
    }

    Invoke-PicoCommand -SerialPort $serial -Command 'P' `
        -ExpectedPattern 'PONG;CORE=PICORV32;WB=PASS' -CommandTimeoutMs $TimeoutMs | Out-Null
    Invoke-PicoCommand -SerialPort $serial -Command 'I' `
        -ExpectedPattern 'ID=0x52424F54;TARGET=GW5A-25B;ABI=1' -CommandTimeoutMs $TimeoutMs | Out-Null
    Invoke-PicoCommand -SerialPort $serial -Command 'S' `
        -ExpectedPattern 'STATUS=PASS;SCRATCH=0xC0DEC0DE;CYCLE=0x[0-9A-F]{8}' -CommandTimeoutMs $TimeoutMs | Out-Null
    Invoke-PicoCommand -SerialPort $serial -Command '?' `
        -ExpectedPattern 'COMMANDS:P=PING,I=ID,S=STATUS,\?=HELP' -CommandTimeoutMs $TimeoutMs | Out-Null
    Write-Host "PicoRV32 SERIAL DEBUG PASS：$Port @ $BaudRate 8-N-1。" -ForegroundColor Green
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
