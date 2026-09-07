#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$TutorialRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$RawRoot = Join-Path $TutorialRoot 'raw'
$ReportRoot = Join-Path $TutorialRoot 'reports'
$ArchiveManifest = Join-Path $ReportRoot 'archive_manifest.json'

foreach ($path in @($TutorialRoot, $RawRoot, $ReportRoot, $ArchiveManifest)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少教程提取产物：$path" }
}

function Get-Attribute([string]$Attributes, [string]$Name) {
    $pattern = '(?i)\b' + [regex]::Escape($Name) + '="([^"]*)"'
    $match = [regex]::Match($Attributes, $pattern)
    if ($match.Success) { return $match.Groups[1].Value }
    return ''
}

function Get-ChapterFromPath([string]$Path) {
    $match = [regex]::Match($Path, '(?i)[\\/]ch(\d{2})[\\/]')
    if ($match.Success) { return [int]$match.Groups[1].Value }
    return 0
}

function Get-RelativePathSafe([string]$BasePath, [string]$Path) {
    return [IO.Path]::GetRelativePath($BasePath, $Path).Replace('\', '/')
}

$chapterTitles = @{}
foreach ($record in (Get-Content -LiteralPath $ArchiveManifest -Raw | ConvertFrom-Json)) {
    if (-not $chapterTitles.ContainsKey([int]$record.chapter)) {
        $chapterTitles[[int]$record.chapter] = [string]$record.title
    }
}

$projectRecords = [System.Collections.Generic.List[object]]::new()
$fileRecords = [System.Collections.Generic.List[object]]::new()
$moduleRecords = [System.Collections.Generic.List[object]]::new()
$projects = Get-ChildItem -LiteralPath $RawRoot -Recurse -File -Filter '*.gprj' |
    Sort-Object FullName

foreach ($project in $projects) {
    $content = Get-Content -LiteralPath $project.FullName -Raw
    $projectDir = $project.DirectoryName
    $chapter = Get-ChapterFromPath $project.FullName
    $deviceMatch = [regex]::Match($content, '<Device\b(?<attrs>[^>]*)>(?<db>[^<]+)</Device>', 'IgnoreCase')
    $deviceAttrs = if ($deviceMatch.Success) { $deviceMatch.Groups['attrs'].Value } else { '' }
    $deviceName = Get-Attribute $deviceAttrs 'name'
    $devicePn = Get-Attribute $deviceAttrs 'pn'
    $deviceDb = if ($deviceMatch.Success) { $deviceMatch.Groups['db'].Value.Trim() } else { '' }

    $enabledFiles = [System.Collections.Generic.List[object]]::new()
    $missingEnabled = [System.Collections.Generic.List[string]]::new()
    foreach ($fileMatch in [regex]::Matches($content, '<File\b(?<attrs>[^>]*)/?>', 'IgnoreCase')) {
        $attrs = $fileMatch.Groups['attrs'].Value
        $relative = Get-Attribute $attrs 'path'
        $type = Get-Attribute $attrs 'type'
        $enableText = Get-Attribute $attrs 'enable'
        $enabled = $enableText -ne '0'
        if ([string]::IsNullOrWhiteSpace($relative)) { continue }
        $nativeRelative = $relative.Replace('/', [IO.Path]::DirectorySeparatorChar)
        $resolved = [IO.Path]::GetFullPath((Join-Path $projectDir $nativeRelative))
        $exists = Test-Path -LiteralPath $resolved -PathType Leaf
        $fileRecord = [pscustomobject]@{
            chapter = $chapter
            project = $project.BaseName
            project_file = Get-RelativePathSafe $TutorialRoot $project.FullName
            relative_path = $relative
            type = $type
            enabled = $enabled
            exists = $exists
            bytes = if ($exists) { (Get-Item -LiteralPath $resolved).Length } else { 0 }
            sha256 = if ($exists) { (Get-FileHash -Algorithm SHA256 -LiteralPath $resolved).Hash.ToLowerInvariant() } else { '' }
        }
        $fileRecords.Add($fileRecord)
        if ($enabled) {
            $enabledFiles.Add($fileRecord)
            if (-not $exists) { $missingEnabled.Add($relative) }
        }
    }

    $hdlFiles = @($enabledFiles | Where-Object { $_.type -match '(?i)verilog|vhdl' -and $_.exists })
    foreach ($hdlFile in $hdlFiles) {
        $resolvedHdl = [IO.Path]::GetFullPath((Join-Path $projectDir $hdlFile.relative_path.Replace('/', [IO.Path]::DirectorySeparatorChar)))
        $hdlText = Get-Content -LiteralPath $resolvedHdl -Raw -ErrorAction Stop
        # This is an inventory parser, not a compiler. Remove comments first to avoid
        # counting commented-out module declarations.
        $withoutComments = [regex]::Replace($hdlText, '(?s)/\*.*?\*/|//[^\r\n]*', '')
        foreach ($moduleMatch in [regex]::Matches($withoutComments, '(?im)^\s*module\s+([A-Za-z_][A-Za-z0-9_$]*)\b')) {
            $moduleRecords.Add([pscustomobject]@{
                chapter = $chapter
                project = $project.BaseName
                module = $moduleMatch.Groups[1].Value
                source = $hdlFile.relative_path
                project_file = Get-RelativePathSafe $TutorialRoot $project.FullName
            })
        }
    }

    $projectUser = "$($project.FullName).user"
    $resultFs = @(Get-ChildItem -LiteralPath $projectDir -Recurse -File -Filter '*.fs' -ErrorAction SilentlyContinue)
    $resultSyn = @(Get-ChildItem -LiteralPath $projectDir -Recurse -File -Filter '*_syn.rpt.html' -ErrorAction SilentlyContinue)
    $resultPnr = @(Get-ChildItem -LiteralPath $projectDir -Recurse -File -Filter '*.rpt.html' -ErrorAction SilentlyContinue |
        Where-Object { $_.DirectoryName -match '(?i)[\\/]pnr$' })
    $types = @($enabledFiles | Group-Object type | ForEach-Object { "$($_.Name):$($_.Count)" })

    $projectRecords.Add([pscustomobject]@{
        chapter = $chapter
        chapter_title = if ($chapterTitles.ContainsKey($chapter)) { $chapterTitles[$chapter] } else { '' }
        project = $project.BaseName
        project_file = Get-RelativePathSafe $TutorialRoot $project.FullName
        device_name = $deviceName
        device_pn = $devicePn
        device_db = $deviceDb
        enabled_files = $enabledFiles.Count
        enabled_hdl = $hdlFiles.Count
        enabled_constraints = @($enabledFiles | Where-Object { $_.type -match '(?i)cst|sdc' }).Count
        enabled_ip = @($enabledFiles | Where-Object { $_.type -match '(?i)ipc|ipx|gao' }).Count
        file_types = $types -join ';'
        missing_enabled_files = $missingEnabled.Count
        missing_paths = $missingEnabled -join ';'
        module_declarations = @($moduleRecords | Where-Object project_file -eq (Get-RelativePathSafe $TutorialRoot $project.FullName)).Count
        original_user_state = Test-Path -LiteralPath $projectUser
        original_syn_reports = $resultSyn.Count
        original_pnr_reports = $resultPnr.Count
        original_bitstreams = $resultFs.Count
    })
}

$uniqueModules = $moduleRecords |
    Group-Object module |
    Sort-Object Name |
    ForEach-Object {
        $chapters = @($_.Group.chapter | Sort-Object -Unique)
        $sources = @($_.Group | ForEach-Object { "$($_.project_file):$($_.source)" } | Sort-Object -Unique)
        [pscustomobject]@{
            module = $_.Name
            declarations = $_.Count
            chapters = ($chapters | ForEach-Object { '{0:D2}' -f $_ }) -join ','
            distinct_sources = $sources.Count
            sources = $sources -join ';'
        }
    }

$projectRecords | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'project_manifest.json') -Encoding utf8
$projectRecords | Export-Csv -LiteralPath (Join-Path $ReportRoot 'project_manifest.csv') -NoTypeInformation -Encoding utf8
$fileRecords | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'project_files.json') -Encoding utf8
$fileRecords | Export-Csv -LiteralPath (Join-Path $ReportRoot 'project_files.csv') -NoTypeInformation -Encoding utf8
$moduleRecords | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'module_declarations.json') -Encoding utf8
$moduleRecords | Export-Csv -LiteralPath (Join-Path $ReportRoot 'module_declarations.csv') -NoTypeInformation -Encoding utf8
$uniqueModules | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'unique_modules.json') -Encoding utf8
$uniqueModules | Export-Csv -LiteralPath (Join-Path $ReportRoot 'unique_modules.csv') -NoTypeInformation -Encoding utf8

$missingProjects = @($projectRecords | Where-Object missing_enabled_files -gt 0)
$deviceGroups = @($projectRecords | Group-Object device_name, device_pn, device_db)
$originalSyn = @($projectRecords | Where-Object original_syn_reports -gt 0).Count
$originalPnr = @($projectRecords | Where-Object original_pnr_reports -gt 0).Count
$originalFs = @($projectRecords | Where-Object original_bitstreams -gt 0).Count

Write-Host "PROJECT_AUDIT projects=$($projectRecords.Count) missing_projects=$($missingProjects.Count) enabled_files=$($fileRecords.Where({$_.enabled}).Count)"
Write-Host "MODULE_AUDIT declarations=$($moduleRecords.Count) unique_modules=$($uniqueModules.Count)"
Write-Host "ORIGINAL_RESULTS syn_projects=$originalSyn pnr_projects=$originalPnr bitstream_projects=$originalFs"
foreach ($group in $deviceGroups) {
    Write-Host "DEVICE_GROUP count=$($group.Count) device=$($group.Name)"
}
if ($missingProjects.Count -gt 0) {
    Write-Warning "$($missingProjects.Count) 个工程存在启用但缺失的文件；详见 project_manifest.csv。"
}
