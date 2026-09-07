param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$timing = Import-Csv (Join-Path $Root 'reports/pnr_timing_audit.csv') | Where-Object kind -eq 'normalized'

$items = @(
    @{ Chapter=48; Project='eth_udp_loopback_rgmii'; Dir='normalized/ch48_udp_pll_compat'; Gprj='eth_udp_loopback_rgmii.gprj'; Top='eth_udp_loopback_rgmii'; Failure='PA2078: 125 MHz PFD exceeds 81.25 MHz; then CPU/SSPI pins are reserved by default'; Fix='PLL IDIV/MDIV 1/7 -> 2/14; enable CPU and SSPI pins as GPIO'; Log='reports/normalized_logs/ch48_udp_pll_and_config_pins_pnr.log' },
    @{ Chapter=54; Project='udp_send_rgmii'; Dir='normalized/ch54_udp_send_config_pin_compat'; Gprj='udp_send_rgmii.gprj'; Top='eth_send_test_rgmii'; Failure='PR2017/PR2028: RGMII pin is reserved as CPU/SSPI'; Fix='enable CPU and SSPI pins as GPIO; no RTL change'; Log='reports/normalized_logs/ch54_udp_send_config_pin_compat_pnr.log' },
    @{ Chapter=56; Project='eth_udp_rx_rgmii'; Dir='normalized/ch56_eth_udp_rx_pll_compat'; Gprj='eth_udp_rx_rgmii.gprj'; Top='eth_udp_rx_rgmii_test'; Failure='PA2078: 125 MHz PFD exceeds 81.25 MHz'; Fix='PLL IDIV/MDIV 1/7 -> 2/14; enable CPU and SSPI pins as GPIO'; Log='reports/normalized_logs/ch56_eth_udp_rx_pll_compat_pnr.log' },
    @{ Chapter=61; Project='eth_udp_rgmii_ddr3_tft800x480'; Dir='normalized/ch61_eth_udp_ddr3_tft_pll_compat'; Gprj='eth_udp_rgmii_ddr3_tft800x480.gprj'; Top='eth_udp_rgmii_ddr3_tft800x480'; Failure='PA2078: 125 MHz PFD exceeds 81.25 MHz'; Fix='PLL IDIV/MDIV 1/7 -> 2/14; enable CPU and SSPI pins as GPIO'; Log='reports/normalized_logs/ch61_eth_udp_ddr3_tft_pll_compat_pnr.log' },
    @{ Chapter=62; Project='ad7606_ddr3_rgmii'; Dir='normalized/ch62_ad7606_ddr3_rgmii_pll_compat'; Gprj='ad7606_ddr3_rgmii.gprj'; Top='ad7606_ddr3_rgmii'; Failure='PA2078: 125 MHz PFD exceeds 81.25 MHz'; Fix='PLL IDIV/MDIV 1/7 -> 2/14; enable CPU and SSPI pins as GPIO'; Log='reports/normalized_logs/ch62_ad7606_ddr3_rgmii_pll_compat_pnr.log' },
    @{ Chapter=63; Project='acm1030_ddr3_rgmii'; Dir='normalized/ch63_acm1030_ddr3_rgmii_pll_compat'; Gprj='acm1030_ddr3_rgmii.gprj'; Top='acm1030_ddr3_rgmii'; Failure='PA2078: 125 MHz PFD exceeds 81.25 MHz'; Fix='PLL IDIV/MDIV 1/7 -> 2/14; enable CPU and SSPI pins as GPIO'; Log='reports/normalized_logs/ch63_acm1030_ddr3_rgmii_pll_compat_pnr.log' }
)

$rows = foreach ($item in $items) {
    $dir = Join-Path $Root $item.Dir
    $bitstream = Get-ChildItem (Join-Path $dir 'impl/pnr') -Filter '*.fs' -File | Select-Object -First 1
    $hash = Get-FileHash $bitstream.FullName -Algorithm SHA256
    $timingRow = $timing | Where-Object { [int]$_.chapter -eq $item.Chapter -and $_.project -eq $item.Project } | Select-Object -First 1
    [pscustomobject]@{
        chapter = $item.Chapter
        project = $item.Project
        top_module = $item.Top
        original_failure = $item.Failure
        normalized_fix = $item.Fix
        normalized_project = ($item.Dir + '/' + $item.Gprj)
        pnr = 'PASS'
        bitstream = [IO.Path]::GetRelativePath($Root, $bitstream.FullName).Replace('\', '/')
        bitstream_bytes = $bitstream.Length
        bitstream_sha256 = $hash.Hash.ToLowerInvariant()
        timing_closure = $timingRow.timing_closure
        setup_violated_endpoints = $timingRow.setup_violated_endpoints
        hold_violated_endpoints = $timingRow.hold_violated_endpoints
        log = $item.Log
    }
}

$csv = Join-Path $Root 'reports/compatibility_pnr_results.csv'
$json = Join-Path $Root 'reports/compatibility_pnr_results.json'
$md = Join-Path $Root 'reports/compatibility_pnr_results.md'
$rows | Export-Csv -NoTypeInformation -Encoding utf8 $csv
$rows | ConvertTo-Json -Depth 4 | Set-Content -Encoding utf8 $json

$lines = [Collections.Generic.List[string]]::new()
$lines.Add('# Normalized compatibility PnR results')
$lines.Add('')
$lines.Add('| Chapter | Project | PnR | Timing | Setup endpoints | Hold endpoints | SHA-256 |')
$lines.Add('|---:|---|---|---|---:|---:|---|')
foreach ($row in $rows) {
    $hashText = $row.bitstream_sha256
    $lines.Add("| $($row.chapter) | $($row.project) | $($row.pnr) | $($row.timing_closure) | $($row.setup_violated_endpoints) | $($row.hold_violated_endpoints) | ``$hashText`` |")
}
$lines.Add('')
$lines.Add('PnR PASS means placement, routing, timing analysis and bitstream generation completed. Timing closure is reported separately.')
[IO.File]::WriteAllLines($md, $lines, [Text.UTF8Encoding]::new($false))
$rows | Format-Table chapter,project,pnr,timing_closure,setup_violated_endpoints,hold_violated_endpoints -AutoSize
