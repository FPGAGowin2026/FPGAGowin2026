#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$GowinShell = 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'
)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$tclScript = Join-Path $PSScriptRoot 'run_gowin_project.tcl'

if (-not (Test-Path -LiteralPath $GowinShell -PathType Leaf)) {
    throw "未找到 Gowin Shell：$GowinShell"
}

$projects = Get-ChildItem -LiteralPath $root -Directory |
    ForEach-Object { Get-ChildItem -LiteralPath $_.FullName -Filter 'sensor_example.gprj' -File -ErrorAction SilentlyContinue } |
    Sort-Object FullName

$failures = [System.Collections.Generic.List[string]]::new()
foreach ($projectFile in $projects) {
    $projectDir = Split-Path $projectFile.FullName -Parent
    $projectName = Split-Path $projectDir -Leaf
    $reportDir = Join-Path $projectDir 'reports'
    $logFile = Join-Path $reportDir 'gowin.log'
    New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
    Write-Host "Gowin: $projectName" -ForegroundColor Cyan
    $buildStartedUtc = (Get-Date).ToUniversalTime().AddSeconds(-2)
    $output = & $GowinShell $tclScript $projectFile.FullName 2>&1
    $exitCode = $LASTEXITCODE
    $output | Set-Content -LiteralPath $logFile -Encoding UTF8
    $logText = $output -join [Environment]::NewLine
    if ($exitCode -ne 0 -or
        $logText -match '(?im)^\s*(ERROR|FATAL)\b' -or
        $logText -match '(?im)completed with errors' -or
        $logText -notmatch 'GowinSynthesis finish' -or
        $logText -notmatch 'Placement and routing completed' -or
        $logText -notmatch 'Bitstream generation completed') {
        $failures.Add("${projectName}: Gowin 综合或布局布线失败")
        continue
    }
    $synthesisReport = Get-Item -LiteralPath (Join-Path $projectDir 'impl\gwsynthesis\sensor_example_syn.rpt.html') -ErrorAction SilentlyContinue
    $bitstream = Get-Item -LiteralPath (Join-Path $projectDir 'impl\pnr\sensor_example.fs') -ErrorAction SilentlyContinue
    $timingReport = Get-Item -LiteralPath (Join-Path $projectDir 'impl\pnr\sensor_example_tr_content.html') -ErrorAction SilentlyContinue
    if (-not $synthesisReport -or -not $bitstream -or -not $timingReport) {
        $failures.Add("${projectName}: 未找到综合报告、时序报告或布局布线位流")
        continue
    }
    if ($synthesisReport.LastWriteTimeUtc -lt $buildStartedUtc -or
        $bitstream.LastWriteTimeUtc -lt $buildStartedUtc -or
        $timingReport.LastWriteTimeUtc -lt $buildStartedUtc) {
        $failures.Add("${projectName}: 构建产物不是本次运行生成")
        continue
    }
    $timingText = Get-Content -LiteralPath $timingReport.FullName -Raw
    $setupMatch = [regex]::Match($timingText, 'Numbers of Setup Violated Endpoints</td>\s*<td>(\d+)')
    $holdMatch = [regex]::Match($timingText, 'Numbers of Hold Violated Endpoints</td>\s*<td>(\d+)')
    if (-not $setupMatch.Success -or -not $holdMatch.Success -or
        [int]$setupMatch.Groups[1].Value -ne 0 -or
        [int]$holdMatch.Groups[1].Value -ne 0) {
        $failures.Add("${projectName}: 时序报告存在 setup/hold 违规或无法解析")
        continue
    }
    Write-Host "PASS $projectName" -ForegroundColor Green
}

if ($projects.Count -eq 0) {
    throw '未发现任何 sensor_example.gprj。'
}
if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Host "FAIL $_" -ForegroundColor Red }
    exit 1
}
Write-Host "全部 $($projects.Count) 个 Gowin 工程综合与布局布线通过。" -ForegroundColor Green
