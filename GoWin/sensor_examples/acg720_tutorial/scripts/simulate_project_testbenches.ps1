#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$Iverilog = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe',
    [string]$Vvp = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe',
    [string]$GowinPrimitiveLibrary = 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\simlib\gw5a\prim_sim.v',
    [int]$TimeoutSeconds = 5,
    [int[]]$Chapter
)

$ErrorActionPreference = 'Stop'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$ProjectFilesCsv = Join-Path $ReportRoot 'project_files.csv'
$SimulationGuard = Join-Path $PSScriptRoot 'sim_guard.v'
$BuildRoot = Join-Path $ReportRoot 'iverilog_build'
$LogRoot = Join-Path $ReportRoot 'iverilog_logs'
$ResultCsv = Join-Path $ReportRoot 'iverilog_results.csv'
foreach ($path in @($TutorialRoot, $ReportRoot, $ProjectFilesCsv, $SimulationGuard, $Iverilog, $Vvp, $GowinPrimitiveLibrary)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少输入或工具：$path" }
}
foreach ($path in @($BuildRoot, $LogRoot)) { New-Item -ItemType Directory -Force -Path $path | Out-Null }

$records = [Collections.Generic.List[object]]::new()
$projectGroups = Import-Csv -LiteralPath $ProjectFilesCsv |
    Group-Object project_file |
    Sort-Object { [int]$_.Group[0].chapter }, Name

foreach ($group in $projectGroups) {
    $chapterNumber = [int]$group.Group[0].chapter
    if ($Chapter.Count -gt 0 -and $Chapter -notcontains $chapterNumber) { continue }
    $projectFull = [IO.Path]::GetFullPath((Join-Path $TutorialRoot $group.Name.Replace('/', [IO.Path]::DirectorySeparatorChar)))
    $projectDir = Split-Path -Parent $projectFull
    $sourceEntries = @($group.Group | Where-Object { $_.type -match '(?i)verilog' -and $_.exists -eq 'True' })
    $sources = @($sourceEntries | ForEach-Object {
        [IO.Path]::GetFullPath((Join-Path $projectDir $_.relative_path.Replace('/', [IO.Path]::DirectorySeparatorChar)))
    } | Sort-Object -Unique)
    if ($sources.Count -eq 0) { continue }

    $testbenches = [Collections.Generic.List[object]]::new()
    foreach ($source in $sources) {
        $sourceText = Get-Content -LiteralPath $source -Raw
        $withoutComments = [regex]::Replace($sourceText, '(?s)/\*.*?\*/|//[^\r\n]*', '')
        foreach ($moduleMatch in [regex]::Matches($withoutComments, '(?im)^\s*module\s+([A-Za-z_][A-Za-z0-9_$]*(?:_tb)|tb)\b')) {
            $testbenches.Add([pscustomobject]@{ module = $moduleMatch.Groups[1].Value; file = $source })
        }
    }
    foreach ($tb in ($testbenches | Sort-Object module, file -Unique)) {
        $identity = "$($group.Name)|$($tb.module)|$($tb.file)"
        $hash = [Convert]::ToHexString(
            [Security.Cryptography.SHA256]::HashData([Text.Encoding]::UTF8.GetBytes($identity))
        ).Substring(0, 10).ToLowerInvariant()
        $key = 'ch{0:D2}_{1}_{2}' -f $chapterNumber, ($tb.module -replace '[^A-Za-z0-9_.-]', '_'), $hash
        $vvpOut = Join-Path $BuildRoot "$key.vvp"
        $compileLog = Join-Path $LogRoot "$key.compile.log"
        $runOut = Join-Path $LogRoot "$key.stdout.log"
        $runErr = Join-Path $LogRoot "$key.stderr.log"
        $includeArgs = @($sources | ForEach-Object { Split-Path -Parent $_ } | Sort-Object -Unique | ForEach-Object { '-I'; $_ })
        $compileSources = [Collections.Generic.List[string]]::new()
        foreach ($source in $sources) { $compileSources.Add($source) }
        if (-not (@($sources | Where-Object { (Split-Path -Leaf $_) -match '(?i)^prim_(sim|tsim)\.v$' }).Count)) {
            $compileSources.Add($GowinPrimitiveLibrary)
        }
        $compileSources.Add($SimulationGuard)

        Write-Host "SIM_COMPILE_BEGIN chapter=$chapterNumber tb=$($tb.module)"
        $started = Get-Date
        Push-Location $projectDir
        try {
            $compileOutput = & $Iverilog -g2005-sv -s $tb.module -s acg720_sim_guard -o $vvpOut @includeArgs @compileSources 2>&1
            $compileCode = $LASTEXITCODE
        } finally {
            Pop-Location
        }
        $compileOutput | Set-Content -LiteralPath $compileLog -Encoding utf8
        $compileStatus = if ($compileCode -eq 0 -and (Test-Path -LiteralPath $vvpOut)) { 'PASS' } else { 'FAIL' }
        $runStatus = 'NOT_RUN'
        $runCode = ''
        if ($compileStatus -eq 'PASS') {
            foreach ($path in @($runOut, $runErr)) { if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path -Force } }
            $process = Start-Process -FilePath $Vvp -ArgumentList @('-n', $vvpOut) -WorkingDirectory $projectDir -NoNewWindow -PassThru -RedirectStandardOutput $runOut -RedirectStandardError $runErr
            $completed = $process.WaitForExit($TimeoutSeconds * 1000)
            if (-not $completed) {
                $process.Kill($true)
                $process.WaitForExit()
                $runStatus = 'TIMEOUT'
                $runCode = $process.ExitCode
            } else {
                $runCode = $process.ExitCode
                $stderrText = if (Test-Path -LiteralPath $runErr) { [string](Get-Content -LiteralPath $runErr -Raw) } else { '' }
                $runStatus = if ($runCode -eq 0 -and $stderrText -notmatch '(?im)^\s*(ERROR|FATAL)\b') { 'PASS' } else { 'FAIL' }
            }
        }
        $finished = Get-Date
        $diagnostic = if ($compileStatus -eq 'FAIL') {
            ($compileOutput | Select-Object -Last 5) -join ' | '
        } elseif (Test-Path -LiteralPath $runErr) {
            ((Get-Content -LiteralPath $runErr | Select-Object -Last 5) -join ' | ')
        } else { '' }
        $records.Add([pscustomobject]@{
            chapter = $chapterNumber
            project_file = $group.Name
            testbench = $tb.module
            testbench_file = [IO.Path]::GetRelativePath($TutorialRoot, $tb.file).Replace('\', '/')
            source_files = $compileSources.Count
            compile_status = $compileStatus
            run_status = $runStatus
            compile_exit = $compileCode
            run_exit = $runCode
            elapsed_seconds = [math]::Round(($finished - $started).TotalSeconds, 3)
            diagnostic = $diagnostic.Trim()
            compile_log = [IO.Path]::GetRelativePath($TutorialRoot, $compileLog).Replace('\', '/')
            stdout_log = if (Test-Path -LiteralPath $runOut) { [IO.Path]::GetRelativePath($TutorialRoot, $runOut).Replace('\', '/') } else { '' }
            stderr_log = if (Test-Path -LiteralPath $runErr) { [IO.Path]::GetRelativePath($TutorialRoot, $runErr).Replace('\', '/') } else { '' }
        })
        $records | Export-Csv -LiteralPath $ResultCsv -NoTypeInformation -Encoding utf8
        Write-Host "SIM_END chapter=$chapterNumber tb=$($tb.module) compile=$compileStatus run=$runStatus"
    }
}

$records | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'iverilog_results.json') -Encoding utf8
$compilePass = @($records | Where-Object compile_status -eq 'PASS').Count
$runPass = @($records | Where-Object run_status -eq 'PASS').Count
$timeout = @($records | Where-Object run_status -eq 'TIMEOUT').Count
$fail = @($records | Where-Object { $_.compile_status -eq 'FAIL' -or $_.run_status -eq 'FAIL' }).Count
Write-Host "SIM_SUMMARY targets=$($records.Count) compile_pass=$compilePass run_pass=$runPass timeout=$timeout failed=$fail"
