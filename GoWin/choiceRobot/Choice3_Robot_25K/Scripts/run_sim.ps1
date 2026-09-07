#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Iverilog='C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe',
    [string]$Vvp='C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe'
)
$ErrorActionPreference='Stop'
$projectDir=Split-Path -Parent $PSScriptRoot
$buildDir=Join-Path $projectDir 'build\sim'
$reportDir=Join-Path $projectDir 'reports'
New-Item -ItemType Directory -Force -Path $buildDir,$reportDir|Out-Null
if(!(Test-Path -LiteralPath $Iverilog)-or!(Test-Path -LiteralPath $Vvp)){throw '未找到 Icarus Verilog'}
$control=@(
  'src/choice3_safety_supervisor.v','src/choice3_range_fusion.v','src/choice3_navigation_fsm.v',
  'src/choice3_quadrature_decoder.v','src/choice3_encoder_health.v','src/choice3_tb6612_quad_bridge.v',
  'src/sensors/encoder_velocity_estimator.v','src/sensors/wheel_speed_controller.v','src/choice3_control_core.v'
)|ForEach-Object{Join-Path $projectDir $_}
$tests=@(
  @{Name='tb_choice3_control_core';Sources=$control+@(Join-Path $projectDir 'sim/tb_choice3_control_core.sv')},
  @{Name='tb_choice3_uart_console';Sources=@(
    'src/bringup_uart_rx.v','src/bringup_uart_tx.v','src/choice3_uart_console.v',
    'sim/tb_choice3_uart_console.sv')|ForEach-Object{Join-Path $projectDir $_}},
  @{Name='tb_choice3_video_pipeline';Sources=@(
    'src/sensors/ov5640_dvp_capture.v','src/choice3_camera_double_buffer.v','src/choice3_attitude_osd.v',
    'src/choice3_vga_640x480.v','src/choice3_tmds_encoder.v','sim/tb_choice3_video_pipeline.sv')|ForEach-Object{Join-Path $projectDir $_}},
  @{Name='tb_choice3_sensor_guards';Sources=@(
    'src/choice3_imu_sample_guard.v','src/choice3_event_freshness.v','sim/tb_choice3_sensor_guards.sv')|ForEach-Object{Join-Path $projectDir $_}},
  @{Name='tb_choice3_attitude_osd';Sources=@(
    'src/choice3_attitude_degrees.v','src/choice3_attitude_osd.v','sim/tb_choice3_attitude_osd.sv')|ForEach-Object{Join-Path $projectDir $_}},
  @{Name='tb_choice3_demo_system';Sources=$control+(@(
    'src/choice3_demo_stimulus.v','src/softcore/softcore_service_watchdog.v',
    'src/softcore/choice3_robot_service_regs.v','sim/tb_choice3_demo_system.sv')|ForEach-Object{Join-Path $projectDir $_})}
)
$discoveredTests=@(Get-ChildItem -LiteralPath (Join-Path $projectDir 'sim') -Filter 'tb_*.sv' -File |
  ForEach-Object {$_.BaseName} | Sort-Object)
$declaredTests=@($tests.Name | Sort-Object)
if(($discoveredTests -join "`n") -ne ($declaredTests -join "`n")){
  throw "仿真清单与 sim/tb_*.sv 不一致。发现：$($discoveredTests -join ', ')；声明：$($declaredTests -join ', ')"
}
$log=Join-Path $reportDir 'simulation.log'
"Choice3 25K regression $(Get-Date -Format o)"|Set-Content -LiteralPath $log
foreach($test in $tests){
  $out=Join-Path $buildDir ($test.Name+'.vvp')
  & $Iverilog -g2012 -Wall -s $test.Name -o $out @($test.Sources) 2>&1|Tee-Object -FilePath $log -Append
  if($LASTEXITCODE-ne 0){throw "$($test.Name) 编译失败"}
  $runText=(& $Vvp $out 2>&1|Tee-Object -FilePath $log -Append|Out-String)
  if($LASTEXITCODE-ne 0){throw "$($test.Name) 仿真失败"}
  $count=([regex]::Matches($runText,"(?m)^PASS: $($test.Name)\r?$" )).Count
  if($count-ne 1){throw "$($test.Name) PASS 标记数量为 $count"}
}
$firmwareBuild=Join-Path $projectDir 'firmware/build'
python (Join-Path $projectDir 'firmware/build_robot_firmware.py') --out-dir $firmwareBuild 2>&1|Tee-Object -FilePath $log -Append
if($LASTEXITCODE-ne 0){throw 'PicoRV32 固件解释执行自检失败'}
$generatedHex=Join-Path $firmwareBuild 'picorv32_selftest.hex'
$itcmHex=Join-Path $projectDir 'firmware/picorv32_selftest.hex'
$generatedArtifacts=@('picorv32_selftest.hex','picorv32_selftest.json','picorv32_selftest.lst')
foreach($artifact in $generatedArtifacts){
  $source=Join-Path $firmwareBuild $artifact
  $destination=Join-Path $projectDir "firmware/$artifact"
  Copy-Item -LiteralPath $source -Destination $destination -Force
  if((Get-FileHash -Algorithm SHA256 -LiteralPath $source).Hash-ne
     (Get-FileHash -Algorithm SHA256 -LiteralPath $destination).Hash){
    throw "固件交付件复制校验失败：$artifact"
  }
}
Write-Host 'PASS: 25K RTL + 固件全部回归通过'
