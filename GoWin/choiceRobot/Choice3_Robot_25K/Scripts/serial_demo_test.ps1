#requires -Version 7.3
[CmdletBinding()]
param([string]$Port='COM5',[int]$BaudRate=115200,[int]$TimeoutMs=18000)
$ErrorActionPreference='Stop'
$projectDir=Split-Path -Parent $PSScriptRoot
$logPath=Join-Path $projectDir 'reports\board_uart_demo.log'
$serial=[System.IO.Ports.SerialPort]::new($Port,$BaudRate,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.Encoding=[Text.Encoding]::ASCII;$serial.ReadTimeout=100;$serial.WriteTimeout=500
$serial.DtrEnable=$false;$serial.RtsEnable=$false
$lines=[System.Collections.Generic.List[string]]::new()
function Invoke-Command([string]$command,[string]$expected,[int]$waitMs=2500){
    $serial.DiscardInBuffer();$serial.Write($command);$deadline=[DateTime]::UtcNow.AddMilliseconds($waitMs);$rx=''
    while([DateTime]::UtcNow-lt$deadline){$rx+=$serial.ReadExisting();if($rx.Contains($expected)){$script:lines.Add("$command -> $($rx.Trim())");return $rx};Start-Sleep -Milliseconds 20}
    throw "命令 $command 未收到 $expected，实际：$rx"
}
try{
    $serial.Open();Start-Sleep -Milliseconds 300
    $null=Invoke-Command 'P' 'PONG;CHOICE3=READY;CORE=PICORV32'
    $null=Invoke-Command 'I' 'ID=0x52424F54;TARGET=GW5A-25B;ABI=2'
    # 上电时 CPU 看门狗与传感器模型尚未就绪，会留下历史诊断位。
    # 就绪后显式清除，再执行正常的武装和路径验收。
    $null=Invoke-Command 'X' 'ACK;FAULT_CLEAR'
    $null=Invoke-Command 'A' 'ACK;ARM'
    Start-Sleep -Milliseconds 100
    $null=Invoke-Command 'G' 'ACK;GO=WAYPOINTS'
    $deadline=[DateTime]::UtcNow.AddMilliseconds($TimeoutMs);$done=$false
    while([DateTime]::UtcNow-lt$deadline-and-not$done){
        $rx=Invoke-Command 'V' 'DEMO=0x' 2000
        if($rx.Contains('DEMO=0x000000FF;EXPECT=000000FF')){$done=$true;break}
        Start-Sleep -Milliseconds 200
    }
    if(-not$done){throw "片上整机场景未在 ${TimeoutMs}ms 内达到 0xFF"}
    $status=Invoke-Command 'S' 'STATUS0=0x' 2500
    if($status-notmatch 'STATUS0=0xFF00[6][A-Fa-f0-9]{3}'){throw "最终健康/导航状态不符合预期：$status"}
    $lines.Add("SERIAL CHOICE3 DEMO PASS: $Port @ $BaudRate;ABI=2;DEMO=0xFF;PHYSICAL_DRIVE=0 (DEMO bit7 + hard-zero Demo top)")
}finally{if($serial.IsOpen){$serial.Close()};$serial.Dispose()}
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $logPath)|Out-Null
$lines|Set-Content -LiteralPath $logPath -Encoding utf8
$lines|ForEach-Object{Write-Host $_ -ForegroundColor Green}
Write-Host "证据日志：$logPath" -ForegroundColor Cyan
