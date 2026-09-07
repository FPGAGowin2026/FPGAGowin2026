#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$PortName = 'COM5',
    [int]$BaudRate = 115200,
    [int]$TimeoutMs = 3000,
    [string]$ExpectedHealth = 'F0',
    [string]$ExpectedFault = '9F',
    [string]$ExpectedVlrs = '00',
    [switch]$SkipExactSensorFields
)

$ErrorActionPreference = 'Stop'
$StatusPattern = 'STATUS;ARM=([01]);HEALTH=([0-9A-F]{2});FAULT=([0-9A-F]{2});NAV=([0-9A-F]);VLRS=([0-9A-F]{2});DRV=([01])'

function Assert-True([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw "UART BOARD TEST FAIL：$Message" }
}

function Read-UntilPatterns {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string[]]$Patterns,
        [int]$WaitMs
    )
    $buffer = ''
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $WaitMs) {
        $chunk = $Serial.ReadExisting()
        if ($chunk.Length -gt 0) { $buffer += $chunk }
        $allFound = $true
        foreach ($pattern in $Patterns) {
            if ($buffer -notmatch $pattern) { $allFound = $false; break }
        }
        if ($allFound) { return $buffer }
        Start-Sleep -Milliseconds 10
    }
    $shown = $buffer.Replace("`r", '<CR>').Replace("`n", '<LF>')
    throw "等待串口响应超时；已收到：$shown"
}

function Invoke-UartCommand {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [char]$Command,
        [string[]]$Patterns
    )
    $Serial.DiscardInBuffer()
    $Serial.Write([string]$Command)
    $response = Read-UntilPatterns -Serial $Serial -Patterns $Patterns -WaitMs $TimeoutMs
    $shown = $response.Replace("`r", '<CR>').Replace("`n", '<LF>')
    Write-Host "$Command <= $shown"
    return $response
}

function Assert-SafeStatus([string]$Text, [string]$Label) {
    $matches = [regex]::Matches($Text, $StatusPattern)
    Assert-True ($matches.Count -gt 0) "$Label 未收到合法 STATUS 行"
    $status = $matches[$matches.Count - 1]
    Assert-True ($status.Groups[1].Value -eq '0') "$Label ARM 不是 0"
    Assert-True ($status.Groups[6].Value -eq '0') "$Label DRV/STBY 不是 0"
    if (-not $SkipExactSensorFields) {
        Assert-True ($status.Groups[2].Value -eq $ExpectedHealth.ToUpperInvariant()) "$Label HEALTH 与无传感器基准不符"
        Assert-True ($status.Groups[3].Value -eq $ExpectedFault.ToUpperInvariant()) "$Label FAULT 与无传感器基准不符"
        Assert-True ($status.Groups[5].Value -eq $ExpectedVlrs.ToUpperInvariant()) "$Label VLRS 与无传感器基准不符"
    }
}

$serial = [System.IO.Ports.SerialPort]::new($PortName, $BaudRate,
    [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.Encoding = [System.Text.Encoding]::ASCII
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 100
$serial.WriteTimeout = 1000

try {
    Write-Host "choice3 board UART safety test $(Get-Date -Format o)"
    $serial.Open()
    Start-Sleep -Milliseconds 250
    $serial.DiscardInBuffer()

    [void](Invoke-UartCommand -Serial $serial -Command 'P' -Patterns @('PONG;CHOICE3=READY'))
    $status0 = Invoke-UartCommand -Serial $serial -Command 'S' -Patterns @($StatusPattern)
    Assert-SafeStatus $status0 '初始状态'

    [void](Invoke-UartCommand -Serial $serial -Command 'A' `
        -Patterns @('ACK;ARM', 'ERR;ARM_REJECTED'))
    $status1 = Invoke-UartCommand -Serial $serial -Command 'S' -Patterns @($StatusPattern)
    Assert-SafeStatus $status1 '拒绝武装后'

    [void](Invoke-UartCommand -Serial $serial -Command 'F' -Patterns @('ACK;MANUAL'))
    $status2 = Invoke-UartCommand -Serial $serial -Command 'S' -Patterns @($StatusPattern)
    Assert-SafeStatus $status2 '未武装手动命令后'

    Write-Host "PASS: choice3 UART 板级安全门禁通过（端口=$PortName；脚本未烧录）"
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
