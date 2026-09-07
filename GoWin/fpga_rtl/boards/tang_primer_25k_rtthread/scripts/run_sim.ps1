#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Iverilog = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe',
    [string]$Vvp = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe'
)

$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectDir 'build'
$reportDir = Join-Path $projectDir 'reports'
$vvpFile = Join-Path $buildDir 'tb_tang_primer_25k_rtthread_top.vvp'
$logFile = Join-Path $reportDir 'simulation.log'

New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
New-Item -ItemType Directory -Force -Path $reportDir | Out-Null

foreach ($tool in @($Iverilog, $Vvp)) {
    if (-not (Test-Path -LiteralPath $tool -PathType Leaf)) {
        throw "未找到 Icarus Verilog 工具：$tool"
    }
}

$sources = @(
    (Join-Path $projectDir 'sim\gowin_gorv32_stub.sv'),
    (Join-Path $projectDir 'src\tang_primer_25k_rtthread_top.v'),
    (Join-Path $projectDir 'sim\tb_tang_primer_25k_rtthread_top.sv')
)

$compileOutput = & $Iverilog -g2012 -Wall -s tb_tang_primer_25k_rtthread_top -o $vvpFile @sources 2>&1
$compileCode = $LASTEXITCODE
@('Tang Primer 25K RT-Thread wrapper - Icarus evidence', '--- compile ---') +
    $compileOutput | Set-Content -LiteralPath $logFile -Encoding utf8
if ($compileCode -ne 0) {
    throw "Icarus 编译失败，退出码 $compileCode。"
}

$runOutput = & $Vvp $vvpFile 2>&1
$runCode = $LASTEXITCODE
'--- run ---' | Add-Content -LiteralPath $logFile -Encoding utf8
$runOutput | Tee-Object -FilePath $logFile -Append
$passLine = 'PASS: Tang Primer 25K RT-Thread wrapper POR/UART connectivity'
if ($runCode -ne 0 -or @($runOutput | Where-Object { $_ -eq $passLine }).Count -ne 1) {
    throw "Icarus 仿真失败，退出码 $runCode。"
}

Write-Host "SIMULATION EVIDENCE：$logFile" -ForegroundColor Cyan
