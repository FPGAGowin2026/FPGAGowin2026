#requires -Version 7.3
[CmdletBinding()]
param(
    [ValidateSet('syn', 'pnr')]
    [string]$Flow = 'syn',
    [int[]]$Chapter,
    [string[]]$Project,
    [string]$TopModule = '',
    [switch]$SkipPassed,
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$GowinShell = 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'
)

$ErrorActionPreference = 'Stop'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$ProjectManifest = Join-Path $ReportRoot 'project_manifest.csv'
$Runner = Join-Path $PSScriptRoot 'rebuild_one.tcl'
$LogRoot = Join-Path $ReportRoot "rebuild_${Flow}_logs"
$ResultCsv = Join-Path $ReportRoot "rebuild_${Flow}_results.csv"
$ResultJson = Join-Path $ReportRoot "rebuild_${Flow}_results.json"

foreach ($path in @($TutorialRoot, $ReportRoot, $ProjectManifest, $Runner, $GowinShell)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少输入或工具：$path" }
}
New-Item -ItemType Directory -Force -Path $LogRoot | Out-Null

$targets = @(Import-Csv -LiteralPath $ProjectManifest)
if ($Chapter.Count -gt 0) {
    $targets = @($targets | Where-Object { $Chapter -contains [int]$_.chapter })
}
if ($Project.Count -gt 0) {
    $targets = @($targets | Where-Object { $Project -contains $_.project })
}
$targets = @($targets | Sort-Object {[int]$_.chapter}, project_file)
if ($targets.Count -eq 0) { throw '筛选后没有待构建工程。' }

$records = [System.Collections.Generic.List[object]]::new()
if (Test-Path -LiteralPath $ResultCsv) {
    foreach ($old in (Import-Csv -LiteralPath $ResultCsv)) {
        if (-not ($old.PSObject.Properties.Name -contains 'requested_top')) {
            $old | Add-Member -NotePropertyName requested_top -NotePropertyValue ''
        }
        $records.Add($old)
    }
}

foreach ($target in $targets) {
    $chapterNumber = [int]$target.chapter
    $pathHash = [Convert]::ToHexString(
        [Security.Cryptography.SHA256]::HashData([Text.Encoding]::UTF8.GetBytes($target.project_file))
    ).Substring(0, 8).ToLowerInvariant()
    $key = 'ch{0:D2}_{1}_{2}' -f $chapterNumber, ($target.project -replace '[^A-Za-z0-9_.-]', '_'), $pathHash
    $projectFull = [IO.Path]::GetFullPath((Join-Path $TutorialRoot $target.project_file.Replace('/', [IO.Path]::DirectorySeparatorChar)))
    $logPath = Join-Path $LogRoot "$key.log"

    $priorPass = @($records | Where-Object {
        $_.project_file -eq $target.project_file -and $_.flow -eq $Flow -and $_.status -eq 'PASS'
    }) | Select-Object -Last 1
    if ($SkipPassed -and $priorPass) {
        Write-Host "SKIP_PASS chapter=$chapterNumber project=$($target.project)"
        continue
    }

    Write-Host "BUILD_BEGIN chapter=$chapterNumber project=$($target.project) flow=$Flow"
    $started = Get-Date
    $output = & $GowinShell $Runner $projectFull $Flow $TopModule 2>&1
    $exitCode = $LASTEXITCODE
    $output | Set-Content -LiteralPath $logPath -Encoding utf8
    $finished = Get-Date
    $logText = $output -join "`n"
    $topMatch = [regex]::Match($logText, 'Current top module is "([^"]+)"')
    $top = if ($topMatch.Success) { $topMatch.Groups[1].Value } else { '' }
    $errorLines = @($output | Where-Object {
        $_ -match '(?i)^\s*(ERROR|FATAL)\b|SYN_COMMAND_FAIL|PNR_COMMAND_FAIL|Synthesis failed|Placement and routing failed'
    })
    $synDone = $logText -match 'GowinSynthesis finish' -and $logText -match 'SYN_COMMAND_DONE'
    $pnrDone = if ($Flow -eq 'pnr') {
        $logText -match 'Placement and routing completed' -and $logText -match 'PNR_COMMAND_DONE'
    } else { $true }
    $status = if ($exitCode -eq 0 -and $synDone -and $pnrDone -and $errorLines.Count -eq 0) { 'PASS' } else { 'FAIL' }

    # Keep only the latest record for a project/flow, so interrupted runs remain resumable.
    $newRecords = [System.Collections.Generic.List[object]]::new()
    foreach ($record in $records) {
        if (-not ($record.project_file -eq $target.project_file -and $record.flow -eq $Flow)) {
            $newRecords.Add($record)
        }
    }
    $records = $newRecords
    $records.Add([pscustomobject]@{
        chapter = $chapterNumber
        project = $target.project
        project_file = $target.project_file
        device_name = $target.device_name
        device_pn = $target.device_pn
        flow = $Flow
        requested_top = $TopModule
        top_module = $top
        status = $status
        exit_code = $exitCode
        elapsed_seconds = [math]::Round(($finished - $started).TotalSeconds, 3)
        error_lines = $errorLines.Count
        diagnostic = ($errorLines | Select-Object -First 5) -join ' | '
        started_at = $started.ToString('o')
        finished_at = $finished.ToString('o')
        log = [IO.Path]::GetRelativePath($TutorialRoot, $logPath).Replace('\', '/')
    })
    $records | Sort-Object {[int]$_.chapter}, project_file, flow |
        Export-Csv -LiteralPath $ResultCsv -NoTypeInformation -Encoding utf8
    $records | Sort-Object {[int]$_.chapter}, project_file, flow |
        ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $ResultJson -Encoding utf8
    Write-Host "BUILD_END chapter=$chapterNumber project=$($target.project) flow=$Flow status=$status top=$top seconds=$([math]::Round(($finished - $started).TotalSeconds, 1))"
}

$current = @($records | Where-Object flow -eq $Flow)
$pass = @($current | Where-Object status -eq 'PASS').Count
$fail = @($current | Where-Object status -eq 'FAIL').Count
Write-Host "BATCH_SUMMARY flow=$Flow recorded=$($current.Count) pass=$pass fail=$fail selected=$($targets.Count)"
if ($fail -gt 0) { exit 1 }
