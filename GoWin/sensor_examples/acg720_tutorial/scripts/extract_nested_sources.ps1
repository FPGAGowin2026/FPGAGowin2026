#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$SevenZip = 'C:\Program Files\7-Zip\7z.exe'
)

$ErrorActionPreference = 'Stop'
$Chapter33 = Join-Path $TutorialRoot 'raw\ch33\ch33_uart_eeprom'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$LogRoot = Join-Path $ReportRoot 'nested_extract_logs'
foreach ($path in @($TutorialRoot, $Chapter33, $ReportRoot, $SevenZip)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少输入或工具：$path" }
}
New-Item -ItemType Directory -Force -Path $LogRoot | Out-Null

# 第 33 章外层包只是一层索引；A.rar 是板级工程，B.rar 是仿真工程。
# 第 64 章中的嵌套包属于 PC 工具、驱动和 FX2 固件，不是 FPGA HDL 源码，
# 因此这里有意不递归展开第 64 章的第三方二进制分发包。
$records = [System.Collections.Generic.List[object]]::new()
foreach ($stem in @('A', 'B')) {
    $archive = Join-Path $Chapter33 "$stem.rar"
    if (-not (Test-Path -LiteralPath $archive)) { throw "第 33 章缺少嵌套源码包：$archive" }
    $destination = Join-Path $Chapter33 "nested_$stem"
    New-Item -ItemType Directory -Force -Path $destination | Out-Null
    $output = & $SevenZip x -y -aoa "-o$destination" $archive 2>&1
    $code = $LASTEXITCODE
    $log = Join-Path $LogRoot "ch33_$stem.log"
    $output | Set-Content -LiteralPath $log -Encoding utf8
    $files = if ($code -eq 0) { @(Get-ChildItem -LiteralPath $destination -Recurse -File) } else { @() }
    $records.Add([pscustomobject]@{
        chapter = 33
        role = if ($stem -eq 'A') { 'board_project' } else { 'simulation_project' }
        archive = [IO.Path]::GetRelativePath($TutorialRoot, $archive).Replace('\', '/')
        sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToLowerInvariant()
        status = if ($code -eq 0) { 'PASS' } else { "FAIL_$code" }
        extracted_files = $files.Count
        hdl_files = @($files | Where-Object Extension -match '^\.(v|sv|vhd|vhdl)$').Count
        gowin_projects = @($files | Where-Object Extension -eq '.gprj').Count
        output = [IO.Path]::GetRelativePath($TutorialRoot, $destination).Replace('\', '/')
        log = [IO.Path]::GetRelativePath($TutorialRoot, $log).Replace('\', '/')
    })
}

$records | Export-Csv -LiteralPath (Join-Path $ReportRoot 'nested_archive_manifest.csv') -NoTypeInformation -Encoding utf8
$records | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'nested_archive_manifest.json') -Encoding utf8
$pass = @($records | Where-Object status -eq 'PASS').Count
Write-Host "NESTED_EXTRACT archives=$($records.Count) pass=$pass hdl=$(($records | Measure-Object hdl_files -Sum).Sum) projects=$(($records | Measure-Object gowin_projects -Sum).Sum)"
if ($pass -ne $records.Count) { exit 1 }
