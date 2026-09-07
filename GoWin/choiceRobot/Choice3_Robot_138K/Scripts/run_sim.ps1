#requires -Version 7.3
[CmdletBinding()]
param(
  [string]$Iverilog='C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe',
  [string]$Vvp='C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe'
)
$ErrorActionPreference='Stop'
$ProjectDir=Split-Path -Parent $PSScriptRoot
$BuildDir=Join-Path $ProjectDir 'build\sim'
$ReportDir=Join-Path $ProjectDir 'reports'
$Log=Join-Path $ReportDir 'simulation.log'
$SummaryFile=Join-Path $ReportDir 'simulation_summary.json'
New-Item -ItemType Directory -Force -Path $BuildDir,$ReportDir|Out-Null
if(!(Test-Path -LiteralPath $Iverilog)-or!(Test-Path -LiteralPath $Vvp)){
  throw '未找到 Icarus Verilog/vvp'
}
function LocalSources([string[]]$Paths){
  @($Paths|ForEach-Object{
    $Resolved=Join-Path $ProjectDir $_
    if(!(Test-Path -LiteralPath $Resolved)){throw "缺少仿真源文件：$Resolved"}
    $Resolved
  })
}
$Control=LocalSources @(
  'src/choice3_safety_supervisor.v','src/choice3_range_fusion.v',
  'src/choice3_navigation_fsm.v','src/choice3_quadrature_decoder.v',
  'src/choice3_encoder_health.v','src/choice3_tb6612_quad_bridge.v',
  'src/sensors/encoder_velocity_estimator.v','src/sensors/wheel_speed_controller.v',
  'src/choice3_control_core.v')
$Tests=@(
  @{Name='tb_choice3_control_core';Sources=$Control+(LocalSources @('sim/tb_choice3_control_core.sv'))},
  @{Name='tb_choice3_navigation_route';Sources=LocalSources @(
    'src/choice3_navigation_fsm.v','sim/tb_choice3_navigation_route.sv')},
  @{Name='tb_choice3_video_pipeline';Sources=LocalSources @(
    'src/sensors/ov5640_dvp_capture.v','src/choice3_camera_double_buffer.v',
    'src/choice3_attitude_osd.v','src/choice3_vga_640x480.v',
    'src/choice3_tmds_encoder.v','sim/tb_choice3_video_pipeline.sv')},
  @{Name='tb_choice3_sensor_guards';Sources=LocalSources @(
    'src/choice3_imu_sample_guard.v','src/choice3_event_freshness.v',
    'sim/tb_choice3_sensor_guards.sv')},
  @{Name='tb_choice3_sensor_protocols';Sources=LocalSources @(
    'src/sensors/jy61p_uart.v','src/sensors/tf_luna_uart.v',
    'sim/tb_choice3_sensor_protocols.sv')},
  @{Name='tb_choice3_attitude_osd';Sources=LocalSources @(
    'src/choice3_attitude_degrees.v','src/choice3_attitude_osd.v',
    'sim/tb_choice3_attitude_osd.sv')},
  @{Name='tb_choice3_external_command_guard';Sources=LocalSources @(
    'src/extension/choice3_external_command_guard.v',
    'src/extension/choice3_command_arbiter.v',
    'sim/tb_choice3_external_command_guard.sv')},
  @{Name='tb_choice3_service_regs';Sources=LocalSources @(
    'src/softcore/choice3_robot_service_regs.v','sim/tb_choice3_service_regs.sv')},
  @{Name='tb_choice3_softcore_watchdog';Sources=LocalSources @(
    'src/softcore/softcore_service_watchdog.v','sim/tb_choice3_softcore_watchdog.sv')}
)

# Every tb_*.sv in this independent folder must be represented exactly once.
$Discovered=@(Get-ChildItem -LiteralPath (Join-Path $ProjectDir 'sim') -File -Filter 'tb_*.sv'|
  ForEach-Object{$_.BaseName}|Sort-Object)
$Declared=@($Tests.Name|Sort-Object)
if(($Discovered-join "`n")-ne($Declared-join "`n")){
  throw "全回归清单与 sim 目录不一致。discovered=$($Discovered-join ',') declared=$($Declared-join ',')"
}
"Choice3 GW5AT-138B Icarus full regression $(Get-Date -Format o)"|
  Set-Content -LiteralPath $Log
$Results=@()
foreach($Test in $Tests){
  $Out=Join-Path $BuildDir ($Test.Name+'.vvp')
  & $Iverilog -g2012 -Wall -s $Test.Name -o $Out @($Test.Sources) 2>&1|
    Tee-Object -FilePath $Log -Append
  if($LASTEXITCODE-ne 0){throw "$($Test.Name) 编译失败"}
  $RunText=(& $Vvp $Out 2>&1|Tee-Object -FilePath $Log -Append|Out-String)
  if($LASTEXITCODE-ne 0){throw "$($Test.Name) 仿真失败"}
  $PassCount=([regex]::Matches($RunText,"(?m)^PASS: $($Test.Name)\r?$" )).Count
  if($PassCount-ne 1){throw "$($Test.Name) PASS 标记数量为 $PassCount"}
  $Results+=[ordered]@{name=$Test.Name;status='PASS';executable=$Out}
}

$FirmwareBuild=Join-Path $ProjectDir 'firmware\build'
python (Join-Path $ProjectDir 'firmware\build_robot_firmware.py') --out-dir $FirmwareBuild 2>&1|
  Tee-Object -FilePath $Log -Append
if($LASTEXITCODE-ne 0){throw 'PicoRV32 固件解释执行自检失败'}
$GeneratedHex=Join-Path $FirmwareBuild 'picorv32_selftest.hex'
$ManifestFile=Join-Path $FirmwareBuild 'picorv32_selftest.json'
$ItcmHex=Join-Path $ProjectDir 'firmware\picorv32_selftest.hex'
foreach($Path in @($GeneratedHex,$ManifestFile)){
  if(!(Test-Path -LiteralPath $Path)){throw "固件缺少产物：$Path"}
}
$Manifest=Get-Content -LiteralPath $ManifestFile -Raw|ConvertFrom-Json
if($Manifest.simulated_boot_output-notmatch 'TARGET=GW5AT-138B'){
  throw '固件自检输出未声明 GW5AT-138B'
}
if([int]$Manifest.uart_identity_abi-ne 3){
  throw '固件 UART 身份 ABI 未与服务寄存器 0x00030000 对齐'
}
$ExpectedIdentity=";TARGET=GW5AT-138B;ABI=3`r`n"
if([string]$Manifest.uart_identity_text-cne $ExpectedIdentity){
  throw '生成固件中的 UART 身份文本未精确声明 ABI=3'
}
if([int]$Manifest.bytes-gt 32768){throw '固件越过 32 KiB ITCM'}
Copy-Item -LiteralPath $GeneratedHex -Destination $ItcmHex -Force
$HexHash=(Get-FileHash -Algorithm SHA256 -LiteralPath $GeneratedHex).Hash.ToLowerInvariant()
if($HexHash-ne (Get-FileHash -Algorithm SHA256 -LiteralPath $ItcmHex).Hash.ToLowerInvariant()){
  throw 'ITCM HEX 复制校验失败'
}
$Summary=[ordered]@{
  generated_at=(Get-Date -Format o);target='GW5AT-138B';test_count=$Tests.Count
  tests=$Results;firmware_status='PASS';firmware_bytes=[int]$Manifest.bytes
  firmware_sha256=$HexHash;log=$Log
}
$Summary|ConvertTo-Json -Depth 5|Set-Content -LiteralPath $SummaryFile -Encoding utf8
Write-Host "PASS: GW5AT-138B Icarus full regression ($($Tests.Count) tests) + firmware"
