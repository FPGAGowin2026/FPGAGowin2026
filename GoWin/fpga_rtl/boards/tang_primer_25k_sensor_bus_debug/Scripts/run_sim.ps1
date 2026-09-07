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
$vvpFile = Join-Path $buildDir 'tb_sensor_bus_debug.vvp'
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
    (Join-Path $projectDir 'src\bus_debug_uart_rx.v'),
    (Join-Path $projectDir 'src\bus_debug_uart_tx.v'),
    (Join-Path $projectDir 'src\sensor_bus_debug_console.v'),
    (Join-Path $sensorRoot 'common\i2c_register8_master.v'),
    (Join-Path $sensorRoot 'ina219_i2c\src\ina219_i2c.v'),
    (Join-Path $sensorRoot 'mpu6050_i2c\src\mpu6050_i2c.v'),
    (Join-Path $sensorRoot 'vl53l0x_i2c\src\vl53l0x_i2c.v'),
    (Join-Path $sensorRoot 'vl53l1x_i2c\src\vl53l1x_i2c.v'),
    (Join-Path $sensorRoot 'icm20602_spi\src\icm20602_spi.v'),
    (Join-Path $sensorRoot 'icm42688p_spi\src\icm42688p_spi.v'),
    (Join-Path $sensorRoot 'imu660ra_spi\src\imu660ra_spi.v'),
    (Join-Path $sensorRoot 'imu660rx_spi\src\imu660rx_spi.v'),
    (Join-Path $sensorRoot 'imu963ra_spi\src\imu963ra_spi.v'),
    (Join-Path $sensorRoot 'lis3mdl_spi\src\lis3mdl_spi.v'),
    (Join-Path $sensorRoot 'menc15a_spi\src\menc15a_spi.v'),
    (Join-Path $sensorRoot 'mt6816_spi\src\mt6816_spi.v'),
    (Join-Path $projectDir 'src\tang_primer_25k_sensor_bus_debug_top.v'),
    (Join-Path $projectDir 'sim\tb_tang_primer_25k_sensor_bus_debug.sv')
)

$compileOutput = & $Iverilog -g2012 -Wall -s tb_tang_primer_25k_sensor_bus_debug -o $vvpFile @sources 2>&1
$compileCode = $LASTEXITCODE
@('Tang Primer 25K sensor bus debug - Icarus evidence', '--- compile ---') +
    $compileOutput | Set-Content -LiteralPath $logFile -Encoding utf8
if ($compileCode -ne 0) {
    throw "Icarus 编译失败，退出码 $compileCode。"
}

$runOutput = & $Vvp $vvpFile 2>&1
$runCode = $LASTEXITCODE
'--- run ---' | Add-Content -LiteralPath $logFile -Encoding utf8
$runOutput | Tee-Object -FilePath $logFile -Append
if ($runCode -ne 0 -or @($runOutput | Where-Object { $_ -match '^PASS:' }).Count -ne 1) {
    throw "Icarus 仿真失败，退出码 $runCode。"
}
Write-Host "SIMULATION EVIDENCE：$logFile" -ForegroundColor Cyan
