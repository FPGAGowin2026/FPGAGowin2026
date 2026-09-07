#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$RawRoot = Join-Path $TutorialRoot 'raw'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$ProjectFilesCsv = Join-Path $ReportRoot 'project_files.csv'
foreach ($path in @($RawRoot, $ReportRoot, $ProjectFilesCsv)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少审计输入：$path" }
}

function Get-Chapter([string]$Path) {
    $match = [regex]::Match($Path, '(?i)[\\/]ch(\d{2})[\\/]')
    if ($match.Success) { return [int]$match.Groups[1].Value }
    return 0
}

$enabledHashes = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach ($entry in (Import-Csv -LiteralPath $ProjectFilesCsv | Where-Object { $_.enabled -eq 'True' -and $_.exists -eq 'True' })) {
    if ($entry.sha256) { [void]$enabledHashes.Add($entry.sha256) }
}

$fileRecords = [Collections.Generic.List[object]]::new()
$declarations = [Collections.Generic.List[object]]::new()
$hdlFiles = Get-ChildItem -LiteralPath $RawRoot -Recurse -File |
    Where-Object { $_.Extension -match '(?i)^\.(v|sv|vh|vhd|vhdl)$' } |
    Sort-Object FullName

foreach ($file in $hdlFiles) {
    $relative = [IO.Path]::GetRelativePath($TutorialRoot, $file.FullName).Replace('\', '/')
    $chapter = Get-Chapter $file.FullName
    $hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $file.FullName).Hash.ToLowerInvariant()
    $text = Get-Content -LiteralPath $file.FullName -Raw
    $withoutComments = [regex]::Replace($text, '(?s)/\*.*?\*/|//[^\r\n]*', '')
    $moduleMatches = @([regex]::Matches($withoutComments, '(?im)^\s*module\s+([A-Za-z_][A-Za-z0-9_$]*)\b'))
    $role = if ($file.BaseName -match '(?i)(^|_)tb$|testbench') {
        'testbench'
    } elseif ($file.BaseName -match '(?i)^prim_(sim|tsim)$' -or $file.FullName -match '(?i)[\\/](gowin_|ipcore|ddr3_memory_interface|fifo|pll|prom|ram|rom)[^\\/]*[\\/]') {
        'generated_or_ip'
    } else {
        'design'
    }
    $fileRecords.Add([pscustomobject]@{
        chapter = $chapter
        file = $relative
        extension = $file.Extension.ToLowerInvariant()
        bytes = $file.Length
        sha256 = $hash
        role = $role
        enabled_by_project = $enabledHashes.Contains($hash)
        module_declarations = $moduleMatches.Count
    })
    foreach ($moduleMatch in $moduleMatches) {
        $declarations.Add([pscustomobject]@{
            chapter = $chapter
            module = $moduleMatch.Groups[1].Value
            role = $role
            file = $relative
            sha256 = $hash
            enabled_by_project = $enabledHashes.Contains($hash)
        })
    }
}

$uniqueModules = $declarations |
    Group-Object module |
    Sort-Object Name |
    ForEach-Object {
        $chapters = @($_.Group.chapter | Sort-Object -Unique)
        $hashes = @($_.Group.sha256 | Sort-Object -Unique)
        $files = @($_.Group.file | Sort-Object -Unique)
        $roles = @($_.Group.role | Sort-Object -Unique)
        [pscustomobject]@{
            module = $_.Name
            declarations = $_.Count
            content_variants = $hashes.Count
            chapters = ($chapters | ForEach-Object { '{0:D2}' -f $_ }) -join ','
            roles = $roles -join ','
            enabled_declarations = @($_.Group | Where-Object enabled_by_project).Count
            files = $files -join ';'
        }
    }

$fileRecords | Export-Csv -LiteralPath (Join-Path $ReportRoot 'all_hdl_files.csv') -NoTypeInformation -Encoding utf8
$fileRecords | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'all_hdl_files.json') -Encoding utf8
$declarations | Export-Csv -LiteralPath (Join-Path $ReportRoot 'all_module_declarations.csv') -NoTypeInformation -Encoding utf8
$declarations | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'all_module_declarations.json') -Encoding utf8
$uniqueModules | Export-Csv -LiteralPath (Join-Path $ReportRoot 'all_unique_modules.csv') -NoTypeInformation -Encoding utf8
$uniqueModules | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'all_unique_modules.json') -Encoding utf8

$designModules = @($uniqueModules | Where-Object { $_.roles -match 'design|generated_or_ip' }).Count
$tbModules = @($uniqueModules | Where-Object { $_.roles -match 'testbench' }).Count
$unreferenced = @($fileRecords | Where-Object { -not $_.enabled_by_project }).Count
Write-Host "ALL_HDL files=$($fileRecords.Count) declarations=$($declarations.Count) unique_modules=$($uniqueModules.Count) design_or_ip_modules=$designModules testbench_modules=$tbModules unreferenced_files=$unreferenced"
