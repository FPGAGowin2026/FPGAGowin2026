#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Port = 'COM5',
    [int]$BaudRate = 115200,
    [int]$TimeoutMs = 2000
)

$ErrorActionPreference = 'Stop'

function Invoke-BringupCommand {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [string]$Command,
        [string]$Expected,
        [int]$CommandTimeoutMs
    )

    $SerialPort.Write($Command)
    $deadline = [DateTime]::UtcNow.AddMilliseconds($CommandTimeoutMs)
    $received = ''
    while ([DateTime]::UtcNow -lt $deadline) {
        $received += $SerialPort.ReadExisting()
        if ($received.Contains($Expected)) {
            Write-Host "PASS '$Command' -> $Expected" -ForegroundColor Green
            return $received
        }
        Start-Sleep -Milliseconds 20
    }
    throw "命令 '$Command' 未在 ${CommandTimeoutMs}ms 内收到 '$Expected'。实际收到：$received"
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
        Write-Host "启动/心跳数据：$($startup.Replace("`r", '<CR>').Replace("`n", '<LF>'))"
    }

    Invoke-BringupCommand -SerialPort $serial -Command 'P' -Expected "PONG`r`n" -CommandTimeoutMs $TimeoutMs | Out-Null
    Invoke-BringupCommand -SerialPort $serial -Command 'I' -Expected "ID=GW5A-LV25MG121`r`n" -CommandTimeoutMs $TimeoutMs | Out-Null
    Invoke-BringupCommand -SerialPort $serial -Command 'S' -Expected "STATUS=OK;CLK=50M;BAUD=115200`r`n" -CommandTimeoutMs $TimeoutMs | Out-Null
    Write-Host "SERIAL DEBUG PASS：$Port @ $BaudRate 8-N-1 双向通信正常。" -ForegroundColor Green
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
