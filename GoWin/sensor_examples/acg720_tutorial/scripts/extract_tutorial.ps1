#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$SourceRoot = 'D:\BaiduNetdiskDownload\小梅哥ACG720国产高云FPGA教学开发板资料',
    [string]$OutputRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$SevenZip = 'C:\Program Files\7-Zip\7z.exe',
    [string]$PdfToText = 'C:\Users\32035\AppData\Local\Microsoft\WinGet\Packages\oschwartz10612.Poppler_Microsoft.Winget.Source_8wekyb3d8bbwe\poppler-25.07.0\Library\bin\pdftotext.exe',
    [string]$PdfInfo = 'C:\Users\32035\.cache\codex-runtimes\codex-primary-runtime\dependencies\native\poppler\Library\bin\pdfinfo.exe'
)

$ErrorActionPreference = 'Stop'
$ExampleRoot = Join-Path $SourceRoot '02_例程源码\GW5AT138K'
$PdfRoot = Join-Path $SourceRoot '01_教材文档\小梅哥高云 ACG720-138&60K通用FPGA开发板自学教程'
$RawRoot = Join-Path $OutputRoot 'raw'
$TextRoot = Join-Path $OutputRoot 'reference_text'
$ReportRoot = Join-Path $OutputRoot 'reports'
$LogRoot = Join-Path $ReportRoot 'extract_logs'

foreach ($path in @($SourceRoot, $ExampleRoot, $PdfRoot, $SevenZip, $PdfToText, $PdfInfo)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少输入或工具：$path" }
}
foreach ($path in @($OutputRoot, $RawRoot, $TextRoot, $ReportRoot, $LogRoot)) {
    New-Item -ItemType Directory -Force -Path $path | Out-Null
}

function Get-ChapterNumber([string]$Name) {
    if ($Name -match '第(\d+)章') { return [int]$Matches[1] }
    return 999
}

function Get-SafeStem([string]$Name) {
    return ($Name -replace '[^A-Za-z0-9_.-]', '_')
}

$chapterDirs = Get-ChildItem -LiteralPath $ExampleRoot -Directory |
    Sort-Object { Get-ChapterNumber $_.Name }
$records = [System.Collections.Generic.List[object]]::new()

foreach ($chapter in $chapterDirs) {
    $chapterNumber = Get-ChapterNumber $chapter.Name
    $chapterKey = 'ch{0:D2}' -f $chapterNumber
    # Serial Hunter 与 Mif Maker 是辅助 PC 软件，不属于 FPGA 例程源码。
    $archives = Get-ChildItem -LiteralPath $chapter.FullName -File |
        Where-Object {
            $_.Extension -match '^\.(rar|zip|7z)$' -and
            $_.BaseName -match '^(ch\d+|acx720_)'
        } | Sort-Object Name

    if ($archives.Count -eq 0) {
        $note = Get-ChildItem -LiteralPath $chapter.FullName -File -Filter '*.txt' |
            Select-Object -First 1
        $records.Add([pscustomobject]@{
            chapter = $chapterNumber
            title = $chapter.Name
            archive = ''
            archive_bytes = 0
            sha256 = ''
            extract_status = 'NO_SOURCE_ARCHIVE'
            extracted_files = 0
            hdl_files = 0
            gowin_projects = 0
            note = if ($note) { (Get-Content -LiteralPath $note.FullName -Raw).Trim() } else { '' }
            output = ''
        })
        continue
    }

    foreach ($archive in $archives) {
        $stem = Get-SafeStem $archive.BaseName
        $archiveOut = Join-Path (Join-Path $RawRoot $chapterKey) $stem
        New-Item -ItemType Directory -Force -Path $archiveOut | Out-Null
        $logPath = Join-Path $LogRoot ("${chapterKey}_${stem}.log")

        $extractOutput = & $SevenZip x -y -aoa "-o$archiveOut" $archive.FullName 2>&1
        $extractCode = $LASTEXITCODE
        @(
            "source=$($archive.FullName)"
            "destination=$archiveOut"
            "exit_code=$extractCode"
            $extractOutput
        ) | Set-Content -LiteralPath $logPath -Encoding utf8

        $files = if ($extractCode -eq 0) {
            @(Get-ChildItem -LiteralPath $archiveOut -File -Recurse -ErrorAction Stop)
        } else { @() }
        $hdl = @($files | Where-Object { $_.Extension -match '^\.(v|sv|vh|vhd|vhdl)$' })
        $projects = @($files | Where-Object { $_.Extension -match '^\.(gprj|gprj\.user)$' })
        $records.Add([pscustomobject]@{
            chapter = $chapterNumber
            title = $chapter.Name
            archive = $archive.Name
            archive_bytes = $archive.Length
            sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive.FullName).Hash.ToLowerInvariant()
            extract_status = if ($extractCode -eq 0) { 'PASS' } else { "FAIL_$extractCode" }
            extracted_files = $files.Count
            hdl_files = $hdl.Count
            gowin_projects = $projects.Count
            note = ''
            output = $archiveOut
        })
    }
}

$pdfRecords = [System.Collections.Generic.List[object]]::new()
$pdfs = Get-ChildItem -LiteralPath $PdfRoot -File -Filter '*.pdf' |
    Sort-Object { Get-ChapterNumber $_.Name }
foreach ($pdf in $pdfs) {
    $chapterNumber = Get-ChapterNumber $pdf.Name
    $chapterKey = 'ch{0:D2}' -f $chapterNumber
    $textPath = Join-Path $TextRoot "$chapterKey.txt"
    $pdfOutput = & $PdfToText -layout $pdf.FullName $textPath 2>&1
    $pdfCode = $LASTEXITCODE
    $infoText = & $PdfInfo $pdf.FullName 2>&1 | Out-String
    $pages = if ($infoText -match '(?m)^Pages:\s+(\d+)') { [int]$Matches[1] } else { 0 }
    $pdfRecords.Add([pscustomobject]@{
        chapter = $chapterNumber
        pdf = $pdf.Name
        pages = $pages
        bytes = $pdf.Length
        sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $pdf.FullName).Hash.ToLowerInvariant()
        text_status = if ($pdfCode -eq 0 -and (Test-Path -LiteralPath $textPath)) { 'PASS' } else { "FAIL_$pdfCode" }
        text_file = $textPath
        diagnostic = ($pdfOutput -join "`n").Trim()
    })
}

$records | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'archive_manifest.json') -Encoding utf8
$records | Export-Csv -LiteralPath (Join-Path $ReportRoot 'archive_manifest.csv') -NoTypeInformation -Encoding utf8
$pdfRecords | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ReportRoot 'pdf_manifest.json') -Encoding utf8
$pdfRecords | Export-Csv -LiteralPath (Join-Path $ReportRoot 'pdf_manifest.csv') -NoTypeInformation -Encoding utf8

$extractPass = @($records | Where-Object extract_status -eq 'PASS').Count
$extractFail = @($records | Where-Object extract_status -like 'FAIL_*').Count
$noSource = @($records | Where-Object extract_status -eq 'NO_SOURCE_ARCHIVE').Count
$pdfPass = @($pdfRecords | Where-Object text_status -eq 'PASS').Count
$hdlTotal = ($records | Measure-Object hdl_files -Sum).Sum
$projectTotal = ($records | Measure-Object gowin_projects -Sum).Sum

Write-Host "EXTRACT_SUMMARY chapters=$($chapterDirs.Count) archives_pass=$extractPass archives_fail=$extractFail no_source=$noSource"
Write-Host "PDF_SUMMARY pdfs=$($pdfRecords.Count) text_pass=$pdfPass"
Write-Host "SOURCE_SUMMARY hdl_files=$hdlTotal gowin_projects=$projectTotal"
if ($extractFail -ne 0 -or $pdfPass -ne $pdfRecords.Count) {
    throw '教程提取存在失败项，请检查 reports 下的清单与日志。'
}
