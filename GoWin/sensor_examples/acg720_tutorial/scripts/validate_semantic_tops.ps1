#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$GowinShell = 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'
)

$ErrorActionPreference = 'Stop'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$TargetsCsv = Join-Path $ReportRoot 'semantic_top_targets.csv'
$Runner = Join-Path $PSScriptRoot 'rebuild_one.tcl'
$LogRoot = Join-Path $ReportRoot 'semantic_top_logs'
foreach ($path in @($TutorialRoot, $ReportRoot, $TargetsCsv, $Runner, $GowinShell)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少输入或工具：$path" }
}
New-Item -ItemType Directory -Force -Path $LogRoot | Out-Null

$records = [Collections.Generic.List[object]]::new()
foreach ($target in (Import-Csv -LiteralPath $TargetsCsv)) {
    $project = [IO.Path]::GetFullPath((Join-Path $TutorialRoot $target.project_file.Replace('/', [IO.Path]::DirectorySeparatorChar)))
    $key = 'ch{0:D2}_{1}' -f [int]$target.chapter, ($target.top_module -replace '[^A-Za-z0-9_.-]', '_')
    $logPath = Join-Path $LogRoot "$key.log"
    Write-Host "SEMANTIC_TOP_BEGIN chapter=$($target.chapter) top=$($target.top_module)"
    $started = Get-Date
    $output = & $GowinShell $Runner $project syn $target.top_module 2>&1
    $exitCode = $LASTEXITCODE
    $output | Set-Content -LiteralPath $logPath -Encoding utf8
    $text = $output -join "`n"
    $errorLines = @($output | Where-Object { $_ -match '(?i)^\s*(ERROR|FATAL)\b|SYN_COMMAND_FAIL|Synthesis failed' })
    $topMatch = [regex]::Match($text, 'Current top module is "([^"]+)"')
    $observedTop = if ($topMatch.Success) { $topMatch.Groups[1].Value } else { '' }
    $status = if (
        $exitCode -eq 0 -and
        $text -match 'GowinSynthesis finish' -and
        $text -match 'SYN_COMMAND_DONE' -and
        $observedTop -eq $target.top_module -and
        $errorLines.Count -eq 0
    ) { 'PASS' } else { 'FAIL' }
    $finished = Get-Date
    $records.Add([pscustomobject]@{
        chapter = [int]$target.chapter
        project_file = $target.project_file
        requested_top = $target.top_module
        observed_top = $observedTop
        status = $status
        exit_code = $exitCode
        elapsed_seconds = [math]::Round(($finished - $started).TotalSeconds, 3)
        errors = $errorLines.Count
        diagnostic = ($errorLines | Select-Object -First 5) -join ' | '
        reason = $target.reason
        log = [IO.Path]::GetRelativePath($TutorialRoot, $logPath).Replace('\', '/')
    })
    Write-Host "SEMANTIC_TOP_END chapter=$($target.chapter) top=$($target.top_module) status=$status"
}

$records | Export-Csv -LiteralPath (Join-Path $ReportRoot 'semantic_top_results.csv') -NoTypeInformation -Encoding utf8
$records | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'semantic_top_results.json') -Encoding utf8
$pass = @($records | Where-Object status -eq 'PASS').Count
$fail = @($records | Where-Object status -eq 'FAIL').Count
Write-Host "SEMANTIC_TOP_SUMMARY targets=$($records.Count) pass=$pass fail=$fail"
if ($fail -gt 0) { exit 1 }
