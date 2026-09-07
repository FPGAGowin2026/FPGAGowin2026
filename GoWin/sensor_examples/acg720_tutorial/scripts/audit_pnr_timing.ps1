param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'

function Get-TimingValue {
    param([string]$Html, [string]$Label)
    $pattern = '<td class="label">' + [regex]::Escape($Label) + '</td>\s*<td>\s*([^<\r\n]+)'
    $match = [regex]::Match($Html, $pattern, [Text.RegularExpressions.RegexOptions]::IgnoreCase)
    if ($match.Success) { return $match.Groups[1].Value.Trim() }
    return $null
}

function Read-TimingReport {
    param([string]$ReportPath)
    if (-not $ReportPath -or -not (Test-Path -LiteralPath $ReportPath)) {
        return [pscustomobject]@{ Paths = $null; Endpoints = $null; Setup = $null; Hold = $null; Closure = 'REPORT_MISSING' }
    }
    $html = [IO.File]::ReadAllText($ReportPath)
    $paths = Get-TimingValue $html 'Numbers of Paths Analyzed'
    $endpoints = Get-TimingValue $html 'Numbers of Endpoints Analyzed'
    $setup = Get-TimingValue $html 'Numbers of Setup Violated Endpoints'
    $hold = Get-TimingValue $html 'Numbers of Hold Violated Endpoints'
    $closure = if ($null -eq $setup -or $null -eq $hold) {
        'SUMMARY_MISSING'
    } elseif ([int]$setup -eq 0 -and [int]$hold -eq 0) {
        'CLEAN'
    } else {
        'VIOLATED'
    }
    [pscustomobject]@{ Paths = $paths; Endpoints = $endpoints; Setup = $setup; Hold = $hold; Closure = $closure }
}

$rawResultsPath = Join-Path $Root 'reports/rebuild_pnr_results.csv'
$rawResults = Import-Csv $rawResultsPath
$rows = foreach ($item in $rawResults) {
    $projectPath = Join-Path $Root $item.project_file
    $projectDir = Split-Path -Parent $projectPath
    $report = Get-ChildItem (Join-Path $projectDir 'impl/pnr') -Filter '*_tr_content.html' -File -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($item.status -eq 'PASS') {
        $timing = Read-TimingReport ($report.FullName)
    } else {
        $timing = [pscustomobject]@{ Paths = $null; Endpoints = $null; Setup = $null; Hold = $null; Closure = 'PNR_FAILED' }
    }
    [pscustomobject]@{
        kind = 'raw'
        chapter = [int]$item.chapter
        project = $item.project
        pnr_status = $item.status
        timing_closure = $timing.Closure
        paths_analyzed = $timing.Paths
        endpoints_analyzed = $timing.Endpoints
        setup_violated_endpoints = $timing.Setup
        hold_violated_endpoints = $timing.Hold
        report = if ($report) { [IO.Path]::GetRelativePath($Root, $report.FullName).Replace('\', '/') } else { '' }
    }
}

$compatItems = @(
    @{ Chapter = 48; Project = 'eth_udp_loopback_rgmii'; Dir = 'normalized/ch48_udp_pll_compat' },
    @{ Chapter = 54; Project = 'udp_send_rgmii'; Dir = 'normalized/ch54_udp_send_config_pin_compat' },
    @{ Chapter = 56; Project = 'eth_udp_rx_rgmii'; Dir = 'normalized/ch56_eth_udp_rx_pll_compat' },
    @{ Chapter = 61; Project = 'eth_udp_rgmii_ddr3_tft800x480'; Dir = 'normalized/ch61_eth_udp_ddr3_tft_pll_compat' },
    @{ Chapter = 62; Project = 'ad7606_ddr3_rgmii'; Dir = 'normalized/ch62_ad7606_ddr3_rgmii_pll_compat' },
    @{ Chapter = 63; Project = 'acm1030_ddr3_rgmii'; Dir = 'normalized/ch63_acm1030_ddr3_rgmii_pll_compat' }
)

$rows += foreach ($item in $compatItems) {
    $dir = Join-Path $Root $item.Dir
    $report = Get-ChildItem (Join-Path $dir 'impl/pnr') -Filter '*_tr_content.html' -File -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    $bitstream = Get-ChildItem (Join-Path $dir 'impl/pnr') -Filter '*.fs' -File -ErrorAction SilentlyContinue | Select-Object -First 1
    $timing = Read-TimingReport ($report.FullName)
    [pscustomobject]@{
        kind = 'normalized'
        chapter = $item.Chapter
        project = $item.Project
        pnr_status = if ($bitstream) { 'PASS' } else { 'FAIL' }
        timing_closure = $timing.Closure
        paths_analyzed = $timing.Paths
        endpoints_analyzed = $timing.Endpoints
        setup_violated_endpoints = $timing.Setup
        hold_violated_endpoints = $timing.Hold
        report = if ($report) { [IO.Path]::GetRelativePath($Root, $report.FullName).Replace('\', '/') } else { '' }
    }
}

$csvPath = Join-Path $Root 'reports/pnr_timing_audit.csv'
$jsonPath = Join-Path $Root 'reports/pnr_timing_audit.json'
$mdPath = Join-Path $Root 'reports/pnr_timing_audit.md'
$rows | Export-Csv -NoTypeInformation -Encoding utf8 $csvPath
$rows | ConvertTo-Json -Depth 4 | Set-Content -Encoding utf8 $jsonPath

$raw = $rows | Where-Object kind -eq 'raw'
$compat = $rows | Where-Object kind -eq 'normalized'
$md = @(
    '# PnR timing audit',
    '',
    "- Raw projects: $($raw.Count)",
    "- Raw PnR pass: $(($raw | Where-Object pnr_status -eq 'PASS').Count)",
    "- Raw timing clean: $(($raw | Where-Object timing_closure -eq 'CLEAN').Count)",
    "- Raw timing violated: $(($raw | Where-Object timing_closure -eq 'VIOLATED').Count)",
    "- Normalized compatibility PnR pass: $(($compat | Where-Object pnr_status -eq 'PASS').Count)/$($compat.Count)",
    "- Normalized compatibility timing clean: $(($compat | Where-Object timing_closure -eq 'CLEAN').Count)",
    "- Normalized compatibility timing violated: $(($compat | Where-Object timing_closure -eq 'VIOLATED').Count)",
    '',
    'A successful PnR/bitstream result is not automatically a timing-closure result. See the CSV for exact endpoint counts.'
)
[IO.File]::WriteAllLines($mdPath, $md, [Text.UTF8Encoding]::new($false))

$rows | Group-Object kind,timing_closure | Select-Object Name,Count | Format-Table -AutoSize
