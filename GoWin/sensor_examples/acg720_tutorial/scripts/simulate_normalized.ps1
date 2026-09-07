#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$Iverilog = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe',
    [string]$Vvp = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe'
)

$ErrorActionPreference = 'Stop'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$BuildRoot = Join-Path $ReportRoot 'normalized_sim_build'
$LogRoot = Join-Path $ReportRoot 'normalized_sim_logs'
$Guard = Join-Path $PSScriptRoot 'sim_guard.v'
foreach ($path in @($TutorialRoot, $ReportRoot, $Guard, $Iverilog, $Vvp)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少输入或工具：$path" }
}
foreach ($path in @($BuildRoot, $LogRoot)) { New-Item -ItemType Directory -Force -Path $path | Out-Null }

$cases = @(
    [pscustomobject]@{
        chapter = 12
        name = 'key_led_original_tb_with_compat_alias'
        top = 'key_led_tb'
        sources = @(
            'raw/ch12/ch12_key_led/src/key_filter.v',
            'raw/ch12/ch12_key_led/src/key_led_top.v',
            'raw/ch12/ch12_key_led/src/key_model.v',
            'raw/ch12/ch12_key_led/src/led_ctrl.v',
            'raw/ch12/ch12_key_led/src/key_led_tb.v',
            'normalized/ch12_key_led_tb_compat/key_led_compat.v'
        )
    },
    [pscustomobject]@{
        chapter = 44
        name = 'dvp_capture_original_tb'
        top = 'DVP_Capture_tb'
        sources = @(
            'raw/ch44/ch44_ov5640_init_DVP/DVP_Capture/DVP_Capture.v',
            'raw/ch44/ch44_ov5640_init_DVP/DVP_Capture/DVP_Capture_tb.v'
        )
    }
)

$records = [Collections.Generic.List[object]]::new()
foreach ($case in $cases) {
    $key = 'ch{0:D2}_{1}' -f [int]$case.chapter, $case.name
    $outputFile = Join-Path $BuildRoot "$key.vvp"
    $compileLog = Join-Path $LogRoot "$key.compile.log"
    $runLog = Join-Path $LogRoot "$key.run.log"
    $sources = @($case.sources | ForEach-Object { [IO.Path]::GetFullPath((Join-Path $TutorialRoot $_.Replace('/', [IO.Path]::DirectorySeparatorChar))) })
    foreach ($source in $sources) { if (-not (Test-Path -LiteralPath $source)) { throw "缺少仿真源码：$source" } }
    $compileOutput = & $Iverilog -g2005-sv -s $case.top -s acg720_sim_guard -o $outputFile @sources $Guard 2>&1
    $compileCode = $LASTEXITCODE
    $compileOutput | Set-Content -LiteralPath $compileLog -Encoding utf8
    $compileStatus = if ($compileCode -eq 0 -and (Test-Path -LiteralPath $outputFile)) { 'PASS' } else { 'FAIL' }
    $runStatus = 'NOT_RUN'
    $runCode = ''
    if ($compileStatus -eq 'PASS') {
        $runOutput = & $Vvp -n $outputFile 2>&1
        $runCode = $LASTEXITCODE
        $runOutput | Set-Content -LiteralPath $runLog -Encoding utf8
        $runStatus = if ($runCode -eq 0 -and ($runOutput -join "`n") -notmatch '(?im)^\s*(ERROR|FATAL)\b') { 'PASS' } else { 'FAIL' }
    }
    $records.Add([pscustomobject]@{
        chapter = [int]$case.chapter
        case = $case.name
        top = $case.top
        sources = $sources.Count
        compile_status = $compileStatus
        run_status = $runStatus
        compile_exit = $compileCode
        run_exit = $runCode
        compile_log = [IO.Path]::GetRelativePath($TutorialRoot, $compileLog).Replace('\', '/')
        run_log = if (Test-Path -LiteralPath $runLog) { [IO.Path]::GetRelativePath($TutorialRoot, $runLog).Replace('\', '/') } else { '' }
    })
    Write-Host "NORMALIZED_SIM chapter=$($case.chapter) case=$($case.name) compile=$compileStatus run=$runStatus"
}

$records | Export-Csv -LiteralPath (Join-Path $ReportRoot 'normalized_sim_results.csv') -NoTypeInformation -Encoding utf8
$records | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'normalized_sim_results.json') -Encoding utf8
$pass = @($records | Where-Object { $_.compile_status -eq 'PASS' -and $_.run_status -eq 'PASS' }).Count
Write-Host "NORMALIZED_SIM_SUMMARY cases=$($records.Count) pass=$pass"
if ($pass -ne $records.Count) { exit 1 }
