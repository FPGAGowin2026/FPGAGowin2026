#requires -Version 7.3
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$expectedProjects = @(
    'digital_ir_proximity',
    'gnss_nmea_uart',
    'gy39_uart',
    'hc_sr04',
    'icm20602_spi',
    'icm42688p_spi',
    'imu660ra_spi',
    'imu660rx_spi',
    'imu963ra_spi',
    'ina219_i2c',
    'itr9606_pulse_speed',
    'jy61p_uart',
    'lis3mdl_spi',
    'maxsonar_uart',
    'mechanical_bumper',
    'menc15a_spi',
    'mpu6050_i2c',
    'mt6816_spi',
    'mt9v03x_dvp',
    'multi_ir_array',
    'ov5640_dvp',
    'ov7725_dvp',
    'quadrature_encoder',
    'scc8660_dvp',
    'tcrt5000',
    'tf_luna_uart',
    'tsl1401_adc_frontend',
    'vl53l0x_i2c',
    'vl53l1x_i2c'
)

$actualProjects = @(Get-ChildItem -LiteralPath $root -Directory |
    Where-Object { Test-Path -LiteralPath (Join-Path $_.FullName 'sensor_example.gprj') } |
    Sort-Object Name | ForEach-Object Name)
$failures = [System.Collections.Generic.List[string]]::new()
if (Compare-Object $expectedProjects $actualProjects) {
    $failures.Add("工程目录清单与预期的 $($expectedProjects.Count) 项不一致")
}

$rows = [System.Collections.Generic.List[object]]::new()
foreach ($name in $expectedProjects) {
    $projectDir = Join-Path $root $name
    $gprjPath = Join-Path $projectDir 'sensor_example.gprj'
    $simLogPath = Join-Path $projectDir 'reports\simulation.log'
    $gowinLogPath = Join-Path $projectDir 'reports\gowin.log'
    $synReportPath = Join-Path $projectDir 'impl\gwsynthesis\sensor_example_syn.rpt.html'
    $pnrReportPath = Join-Path $projectDir 'impl\pnr\sensor_example.rpt.txt'
    $timingPath = Join-Path $projectDir 'impl\pnr\sensor_example_tr_content.html'
    $bitstreamPath = Join-Path $projectDir 'impl\pnr\sensor_example.fs'
    $required = @($gprjPath,$simLogPath,$gowinLogPath,$synReportPath,$pnrReportPath,$timingPath,$bitstreamPath)
    if ($required | Where-Object { -not (Test-Path -LiteralPath $_ -PathType Leaf) }) {
        $failures.Add("${name}: 缺少工程、日志或构建产物")
        continue
    }

    [xml]$projectXml = Get-Content -LiteralPath $gprjPath -Raw
    $device = $projectXml.Project.Device
    if ($device.name -ne 'GW5A-25B' -or
        $device.pn -ne 'GW5A-LV25MG121NC1/I0' -or
        $device.'#text' -ne 'gw5a25b-003') {
        $failures.Add("${name}: 目标器件三元组不正确")
    }

    $simText = Get-Content -LiteralPath $simLogPath -Raw
    if (@([regex]::Matches($simText, '(?m)^PASS:')).Count -ne 1 -or
        $simText -match '(?m)^(FAIL|FATAL|ERROR):') {
        $failures.Add("${name}: 仿真日志不是唯一 PASS 终态")
    }

    $gowinText = Get-Content -LiteralPath $gowinLogPath -Raw
    if ($gowinText -match '(?im)^\s*(ERROR|FATAL)\b' -or
        $gowinText -match '(?im)completed with errors' -or
        $gowinText -notmatch 'GowinSynthesis finish' -or
        $gowinText -notmatch 'Placement and routing completed' -or
        $gowinText -notmatch 'Bitstream generation completed') {
        $failures.Add("${name}: Gowin 日志未显示完整成功")
    }

    $pnrText = Get-Content -LiteralPath $pnrReportPath -Raw
    if ($pnrText -notmatch '<Part Number>: GW5A-LV25MG121NC1/I0' -or
        $pnrText -notmatch '<Device Version>: B' -or
        $pnrText -notmatch '<Timing Constraints File>:(?! ---)') {
        $failures.Add("${name}: PnR 报告器件或时钟约束不正确")
    }

    $timingText = Get-Content -LiteralPath $timingPath -Raw
    $setup = [regex]::Match($timingText, 'Numbers of Setup Violated Endpoints</td>\s*<td>(\d+)')
    $hold = [regex]::Match($timingText, 'Numbers of Hold Violated Endpoints</td>\s*<td>(\d+)')
    $fmax = [regex]::Match($timingText, '<td>([0-9.]+)\(MHz\)</td>\s*<td>\d+</td>\s*<td>TOP</td>')
    if (-not $setup.Success -or -not $hold.Success -or -not $fmax.Success -or
        [int]$setup.Groups[1].Value -ne 0 -or [int]$hold.Groups[1].Value -ne 0) {
        $failures.Add("${name}: setup/hold 时序门禁失败或报告无法解析")
    }

    $logic = [regex]::Match($pnrText, '(?m)^\s*Logic\s+\|\s*(\d+)/').Groups[1].Value
    $registers = [regex]::Match($pnrText, '(?m)^\s*Register\s+\|\s*(\d+)/').Groups[1].Value
    $ports = [regex]::Match($pnrText, '(?m)^\s*I/O Port\s+\|\s*(\d+)/').Groups[1].Value
    if (-not $logic -or -not $registers -or -not $ports) {
        $failures.Add("${name}: 资源统计无法解析")
    } elseif ([int]$ports -gt 69) {
        $failures.Add("${name}: I/O Port=$ports，超过 MG121 可用的 69 个普通 I/O")
    }
    if ((Get-Item -LiteralPath $bitstreamPath).Length -le 0) {
        $failures.Add("${name}: 位流为空")
    }
    $rows.Add([pscustomobject]@{
        Project = $name
        Logic = $logic
        Registers = $registers
        Ports = $ports
        FmaxMHz = $fmax.Groups[1].Value
    })
}

Write-Output '| 工程 | Logic | Register | I/O Port | Actual Fmax (MHz) |'
Write-Output '|---|---:|---:|---:|---:|'
foreach ($row in $rows) {
    Write-Output "| $($row.Project) | $($row.Logic) | $($row.Registers) | $($row.Ports) | $($row.FmaxMHz) |"
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Host "FAIL $_" -ForegroundColor Red }
    exit 1
}
Write-Host "AUDIT PASS：$($expectedProjects.Count)/$($expectedProjects.Count) 仿真、综合、PnR、位流、I/O 上限和 setup/hold 门禁均通过。" -ForegroundColor Green
