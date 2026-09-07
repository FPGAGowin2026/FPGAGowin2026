#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$GowinShell = 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'
)

$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$tclFile = Join-Path $PSScriptRoot 'run_gowin.tcl'
$reportDir = Join-Path $projectDir 'reports'
$logFile = Join-Path $reportDir 'gowin.log'

if (-not (Test-Path -LiteralPath $GowinShell)) {
    throw "未找到 Gowin Shell：$GowinShell"
}

Push-Location $projectDir
try {
    New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
    $gowinOutput = & $GowinShell $tclFile 2>&1
    $gowinCode = $LASTEXITCODE
    $gowinOutput | Tee-Object -FilePath $logFile
    if ($gowinCode -ne 0) {
        throw "Gowin 构建失败，退出码 $gowinCode。"
    }
} finally {
    Pop-Location
}

$baseName = 'tang_primer_25k_rtthread'
$bitstream = Join-Path $projectDir "impl\pnr\$baseName.fs"
$pnrReport = Join-Path $projectDir "impl\pnr\$baseName.rpt.txt"
$timingReport = Join-Path $projectDir "impl\pnr\${baseName}_tr_content.html"
foreach ($required in @($bitstream, $pnrReport, $timingReport)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "缺少构建产物：$required"
    }
}
$pnrText = Get-Content -LiteralPath $pnrReport -Raw
$timingText = Get-Content -LiteralPath $timingReport -Raw
if ($pnrText -notmatch '<Part Number>: GW5A-LV25MG121NC1/I0' -or
    $pnrText -notmatch '<Device Version>: B') {
    throw 'PnR 报告中的目标器件或 Device B 版本不正确。'
}
if ($timingText -notmatch 'Numbers of Setup Violated Endpoints</td>\s*<td>0</td>' -or
    $timingText -notmatch 'Numbers of Hold Violated Endpoints</td>\s*<td>0</td>') {
    throw '时序报告存在 setup/hold 违规或无法识别违规计数。'
}
Write-Host "GOWIN BUILD PASS：$bitstream" -ForegroundColor Green
Write-Host "GOWIN EVIDENCE：$logFile" -ForegroundColor Cyan
