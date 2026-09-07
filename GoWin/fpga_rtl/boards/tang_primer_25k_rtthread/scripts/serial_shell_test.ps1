#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Port = 'COM5',
    [int]$BaudRate = 115200,
    [int]$TimeoutMs = 5000,
    [ValidateRange(1, 1000)]
    [int]$StressRounds = 20,
    [string]$LogPath = ''
)

$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
if (-not $LogPath)
{
    $LogPath = Join-Path $projectDir 'reports\serial_shell.log'
}

function ConvertTo-VisibleSerialText
{
    param([string]$Text)

    return $Text.Replace("`r", '<CR>').Replace("`n", "<LF>`n")
}

function Read-UntilMshPrompt
{
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [int]$CommandTimeoutMs
    )

    $deadline = [DateTime]::UtcNow.AddMilliseconds($CommandTimeoutMs)
    $received = ''
    while ([DateTime]::UtcNow -lt $deadline)
    {
        $received += $SerialPort.ReadExisting()
        if ($received -match '(?m)^msh >')
        {
            return $received
        }
        Start-Sleep -Milliseconds 20
    }

    throw "${CommandTimeoutMs}ms 内未收到 MSH 提示符。实际收到：$(ConvertTo-VisibleSerialText $received)"
}

function Assert-CleanSerialText
{
    param(
        [string]$Text,
        [string]$Context
    )

    $invalid = [System.Collections.Generic.List[string]]::new()
    foreach ($character in $Text.ToCharArray())
    {
        $code = [int]$character
        if ((($code -lt 0x20) -and ($code -notin @(0x0a, 0x0d))) -or
            ($code -gt 0x7e) -or
            ($character -eq '?'))
        {
            $invalid.Add(('0x{0:X2}' -f $code))
        }
    }

    if ($invalid.Count -ne 0)
    {
        $codes = ($invalid | Select-Object -First 16) -join ', '
        throw "$Context 检测到非预期控制字符或非 ASCII 替换字符：$codes。实际收到：$(ConvertTo-VisibleSerialText $Text)"
    }
}

function Invoke-CheckedMshCommand
{
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [string]$Command,
        [string[]]$ExpectedPatterns,
        [int]$CommandTimeoutMs,
        [System.Collections.Generic.List[string]]$LogLines
    )

    # 给 USB-UART 桥一个短暂的静默窗口，避免上一条提示符尾部滞留在驱动缓冲区。
    Start-Sleep -Milliseconds 100
    $SerialPort.DiscardInBuffer()
    $SerialPort.Write("$Command`r")
    $response = Read-UntilMshPrompt $SerialPort $CommandTimeoutMs

    $LogLines.Add("COMMAND=$Command")
    $LogLines.Add('RESPONSE_BEGIN')
    $LogLines.Add((ConvertTo-VisibleSerialText $response))
    $LogLines.Add('RESPONSE_END')

    Assert-CleanSerialText $response "命令 '$Command'"

    if ($response -match 'FATAL:|assertion failed|Unhandled')
    {
        throw "命令 '$Command' 后检测到致命错误：$(ConvertTo-VisibleSerialText $response)"
    }

    foreach ($pattern in $ExpectedPatterns)
    {
        if ($response -notmatch $pattern)
        {
            throw "命令 '$Command' 缺少期望模式 '$pattern'。实际收到：$(ConvertTo-VisibleSerialText $response)"
        }
    }

    $LogLines.Add("PASS=$Command")
    return $response
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
$serial.Handshake = [System.IO.Ports.Handshake]::None

$logLines = [System.Collections.Generic.List[string]]::new()
$logLines.Add("TIMESTAMP=$([DateTimeOffset]::Now.ToString('o'))")
$logLines.Add("PORT=$Port;BAUD=$BaudRate;FORMAT=8-N-1;FLOW=NONE")
$logLines.Add("STRESS_ROUNDS=$StressRounds")
$failure = $null

try
{
    $serial.Open()
    Start-Sleep -Milliseconds 200

    # 不依赖重新上电：空回车同步现有 shell，version 命令可重复验证完整 banner。
    $serial.DiscardInBuffer()
    $serial.Write("`r")
    $sync = Read-UntilMshPrompt $serial $TimeoutMs
    $logLines.Add('SYNC_BEGIN')
    $logLines.Add((ConvertTo-VisibleSerialText $sync))
    $logLines.Add('SYNC_END')
    Assert-CleanSerialText $sync 'shell 同步'

    [void](Invoke-CheckedMshCommand $serial 'version' @(
        'Thread Operating System',
        '\b4\.1\.1\b',
        'Copyright by RT-Thread team'
    ) $TimeoutMs $logLines)

    [void](Invoke-CheckedMshCommand $serial 'list' @(
        'Usage:\s*list \[options\]',
        'thread - list threads',
        'sem - list semaphores'
    ) $TimeoutMs $logLines)

    for ($round = 1; $round -le $StressRounds; $round++)
    {
        [void](Invoke-CheckedMshCommand $serial 'ps' @(
            '(?m)^thread\s+pri\s+status\s+sp\s+stack size max used left tick\s+error\r?$',
            '(?m)^tshell\s+20\s+running\s+0x[0-9a-f]{8}\s+0x00000800\s+\d+%\s+0x[0-9a-f]{8}\s+OK\r?$',
            '(?m)^tidle0\s+31\s+ready\s+0x[0-9a-f]{8}\s+0x00000100\s+\d+%\s+0x[0-9a-f]{8}\s+OK\r?$'
        ) $TimeoutMs $logLines)

        [void](Invoke-CheckedMshCommand $serial 'hello' @(
            '(?m)^hello from Tang Primer 25K / Gowin GoRV32\r?$'
        ) $TimeoutMs $logLines)

        $logLines.Add("STRESS_ROUND=$round/$StressRounds PASS")
    }

    $logLines.Add("SERIAL RT-THREAD SHELL PASS: $Port @ $BaudRate 8-N-1;PS_HELLO_ROUNDS=$StressRounds")
}
catch
{
    $failure = $_
    $logLines.Add("SERIAL RT-THREAD SHELL FAIL: $($_.Exception.Message)")
}
finally
{
    if ($serial.IsOpen)
    {
        $serial.Close()
    }
    $serial.Dispose()

    $logDir = Split-Path -Parent $LogPath
    New-Item -ItemType Directory -Force -Path $logDir | Out-Null
    $logLines | Set-Content -LiteralPath $LogPath -Encoding utf8
}

if ($failure)
{
    Write-Host "串口回归失败，证据日志：$LogPath" -ForegroundColor Red
    throw $failure
}

$logLines | ForEach-Object { Write-Host $_ -ForegroundColor Green }
Write-Host "证据日志：$LogPath" -ForegroundColor Cyan
