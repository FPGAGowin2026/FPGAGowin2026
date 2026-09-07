param(
    [string]$Iverilog = "C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe",
    [string]$Vvp = "C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe"
)
$ErrorActionPreference = "Stop"
$ProjectDir = Split-Path -Parent $PSScriptRoot
$RepoDir = (Resolve-Path (Join-Path $ProjectDir "..\..")).Path
$BuildDir = Join-Path $ProjectDir "build"
$ReportDir = Join-Path $ProjectDir "reports"
New-Item -ItemType Directory -Force $BuildDir,$ReportDir | Out-Null

if (!(Test-Path -LiteralPath $Iverilog) -or !(Test-Path -LiteralPath $Vvp)) {
    throw "未找到 Icarus Verilog；请通过 -Iverilog/-Vvp 指定路径。"
}

$Common = @(
    (Join-Path $ProjectDir "src\choice3_safety_supervisor.v"),
    (Join-Path $ProjectDir "src\choice3_range_fusion.v"),
    (Join-Path $ProjectDir "src\choice3_navigation_fsm.v"),
    (Join-Path $ProjectDir "src\choice3_quadrature_decoder.v"),
    (Join-Path $ProjectDir "src\choice3_encoder_health.v"),
    (Join-Path $ProjectDir "src\choice3_tb6612_quad_bridge.v"),
    (Join-Path $RepoDir "src\motor\encoder_velocity_estimator.v"),
    (Join-Path $RepoDir "src\motor\wheel_speed_controller.v"),
    (Join-Path $ProjectDir "src\choice3_control_core.v")
)

$Tests = @(
    @{Name="tb_choice3_control_core"; Sources=$Common + @((Join-Path $ProjectDir "sim\tb_choice3_control_core.sv"))},
    @{Name="tb_choice3_uart_console"; Sources=@(
        (Join-Path $RepoDir "boards\tang_primer_25k_bringup\src\bringup_uart_rx.v"),
        (Join-Path $RepoDir "boards\tang_primer_25k_bringup\src\bringup_uart_tx.v"),
        (Join-Path $ProjectDir "src\choice3_uart_console.v"),
        (Join-Path $ProjectDir "sim\tb_choice3_uart_console.sv"))},
    @{Name="tb_choice3_video_pipeline"; Sources=@(
        (Join-Path $RepoDir "..\sensor_examples\ov5640_dvp\src\ov5640_dvp_capture.v"),
        (Join-Path $ProjectDir "src\choice3_camera_double_buffer.v"),
        (Join-Path $ProjectDir "src\choice3_vga_640x480.v"),
        (Join-Path $ProjectDir "src\choice3_tmds_encoder.v"),
        (Join-Path $ProjectDir "sim\tb_choice3_video_pipeline.sv"))},
    @{Name="tb_choice3_sensor_guards"; Sources=@(
        (Join-Path $ProjectDir "src\choice3_imu_sample_guard.v"),
        (Join-Path $ProjectDir "src\choice3_event_freshness.v"),
        (Join-Path $ProjectDir "sim\tb_choice3_sensor_guards.sv"))}
)

$Log = Join-Path $ReportDir "simulation.log"
"choice3 Icarus regression $(Get-Date -Format o)" | Set-Content -LiteralPath $Log
foreach ($Test in $Tests) {
    $Output = Join-Path $BuildDir ($Test.Name + ".vvp")
    & $Iverilog -g2012 -Wall -s $Test.Name -o $Output @($Test.Sources) 2>&1 |
        Tee-Object -FilePath $Log -Append
    if ($LASTEXITCODE -ne 0) { throw "$($Test.Name) 编译失败" }
    Push-Location $ProjectDir
    try {
        $RunText = (& $Vvp $Output 2>&1 | Tee-Object -FilePath $Log -Append | Out-String)
    } finally { Pop-Location }
    if ($LASTEXITCODE -ne 0) { throw "$($Test.Name) 仿真失败" }
    $PassCount = ([regex]::Matches($RunText,"(?m)^PASS: $($Test.Name)\r?$")).Count
    if ($PassCount -ne 1) { throw "$($Test.Name) PASS 标记数量为 $PassCount，期望恰好 1" }
}
Write-Host "PASS: choice3 全部 Icarus 门禁通过"
