#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Iverilog = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe',
    [string]$Vvp = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe'
)

$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $projectDir '..\..\..')).Path
$sensorRoot = Join-Path $repoRoot 'sensor_examples'
$buildDir = Join-Path $projectDir 'build'
$vvpFile = Join-Path $buildDir 'tb_sensor_functional_bist.vvp'
$reportDir = Join-Path $projectDir 'reports'
$logFile = Join-Path $reportDir 'simulation.log'

New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
if (-not (Test-Path -LiteralPath $Iverilog -PathType Leaf)) {
    throw "未找到 Icarus Verilog 编译器：$Iverilog"
}
if (-not (Test-Path -LiteralPath $Vvp -PathType Leaf)) {
    throw "未找到 Icarus Verilog 运行器：$Vvp"
}

$sources = @(
    (Join-Path $projectDir 'sim\tb_tang_primer_25k_sensor_functional_bist.sv'),
    (Join-Path $projectDir 'src\func_bist_uart_rx.v'),
    (Join-Path $projectDir 'src\func_bist_uart_tx.v'),
    (Join-Path $projectDir 'src\sensor_functional_bist_console.v'),
    (Join-Path $projectDir 'src\sensor_functional_stimulus.v'),
    (Join-Path $sensorRoot 'digital_ir_proximity\src\digital_ir_proximity.v'),
    (Join-Path $sensorRoot 'gnss_nmea_uart\src\gnss_nmea_uart.v'),
    (Join-Path $sensorRoot 'gy39_uart\src\gy39_uart.v'),
    (Join-Path $sensorRoot 'hc_sr04\src\hc_sr04_controller.v'),
    (Join-Path $sensorRoot 'itr9606_pulse_speed\src\itr9606_pulse_speed.v'),
    (Join-Path $sensorRoot 'jy61p_uart\src\jy61p_uart.v'),
    (Join-Path $sensorRoot 'maxsonar_uart\src\maxsonar_uart.v'),
    (Join-Path $sensorRoot 'mechanical_bumper\src\mechanical_bumper.v'),
    (Join-Path $sensorRoot 'mt9v03x_dvp\src\mt9v03x_dvp.v'),
    (Join-Path $sensorRoot 'multi_ir_array\src\multi_ir_array.v'),
    (Join-Path $sensorRoot 'ov5640_dvp\src\ov5640_dvp_capture.v'),
    (Join-Path $sensorRoot 'ov7725_dvp\src\ov7725_dvp.v'),
    (Join-Path $sensorRoot 'quadrature_encoder\src\quadrature_decoder.v'),
    (Join-Path $sensorRoot 'scc8660_dvp\src\scc8660_dvp.v'),
    (Join-Path $sensorRoot 'tcrt5000\src\tcrt5000_filter.v'),
    (Join-Path $sensorRoot 'tf_luna_uart\src\tf_luna_uart.v'),
    (Join-Path $sensorRoot 'tsl1401_adc_frontend\src\tsl1401_adc_frontend.v'),
    (Join-Path $projectDir 'src\sensor_functional_bist_core.v'),
    (Join-Path $projectDir 'src\tang_primer_25k_sensor_functional_bist_top.v')
)

$compileOutput = & $Iverilog -g2012 -Wall `
    -s tb_tang_primer_25k_sensor_functional_bist -o $vvpFile @sources 2>&1
$compileCode = $LASTEXITCODE
@('Tang Primer 25K sensor functional BIST - Icarus evidence',
  '--- compile ---') + $compileOutput |
    Set-Content -LiteralPath $logFile -Encoding utf8
if ($compileCode -ne 0) {
    throw "Icarus 编译失败，退出码 $compileCode。"
}

$runOutput = & $Vvp $vvpFile 2>&1
$runCode = $LASTEXITCODE
'--- run ---' | Add-Content -LiteralPath $logFile -Encoding utf8
$runOutput | Tee-Object -FilePath $logFile -Append
$passLines = @($runOutput | Where-Object {
    $_ -eq 'PASS: Tang Primer 25K sensor functional BIST (17/17)'
})
if ($runCode -ne 0 -or $passLines.Count -ne 1 -or
    -not ($runOutput -contains 'PASS=0x1ffff OBS=0x1ffff ACT=0x1ffff')) {
    throw "Icarus 仿真未产生唯一 17/17 PASS 证据，退出码 $runCode。"
}
Write-Host "SIMULATION EVIDENCE：$logFile" -ForegroundColor Cyan
