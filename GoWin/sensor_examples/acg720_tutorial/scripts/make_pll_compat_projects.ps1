param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'

$items = @(
    @{
        Name = 'ch56_eth_udp_rx_pll_compat'
        Project = 'raw/ch56/ch56_eth_udp_rx_rgmii/eth_udp_rx_rgmii.gprj'
        Pll = 'raw/ch56/ch56_eth_udp_rx_rgmii/src/gowin_pll/gowin_pll.v'
        Top = 'eth_udp_rx_rgmii_test'
    },
    @{
        Name = 'ch61_eth_udp_ddr3_tft_pll_compat'
        Project = 'raw/ch61/ch61_eth_udp_rgmii_ddr3_tft800x480/eth_udp_rgmii_ddr3_tft800x480/eth_udp_rgmii_ddr3_tft800x480.gprj'
        Pll = 'raw/ch61/ch61_eth_udp_rgmii_ddr3_tft800x480/eth_udp_rgmii_ddr3_tft800x480/src/pll/pll.v'
        Top = 'eth_udp_rgmii_ddr3_tft800x480'
    },
    @{
        Name = 'ch62_ad7606_ddr3_rgmii_pll_compat'
        Project = 'raw/ch62/ch62_ad7606_ddr3_rgmii/ad7606_ddr3_rgmii.gprj'
        Pll = 'raw/ch62/ch62_ad7606_ddr3_rgmii/src/eth_pll/eth_pll.v'
        Top = 'ad7606_ddr3_rgmii'
    },
    @{
        Name = 'ch63_acm1030_ddr3_rgmii_pll_compat'
        Project = 'raw/ch63/ch63_acm1030_ddr3_rgmii/acm1030_ddr3_rgmii.gprj'
        Pll = 'raw/ch63/ch63_acm1030_ddr3_rgmii/src/eth_pll/eth_pll.v'
        Top = 'acm1030_ddr3_rgmii'
    }
)

$manifest = foreach ($item in $items) {
    $projectPath = [IO.Path]::GetFullPath((Join-Path $Root $item.Project))
    $projectDir = Split-Path -Parent $projectPath
    $pllPath = [IO.Path]::GetFullPath((Join-Path $Root $item.Pll))
    $outDir = Join-Path $Root ('normalized/' + $item.Name)
    $outProject = Join-Path $outDir ([IO.Path]::GetFileName($projectPath))
    $outPll = Join-Path $outDir ([IO.Path]::GetFileName($pllPath))

    New-Item -ItemType Directory -Force -Path $outDir | Out-Null

    $pllText = [IO.File]::ReadAllText($pllPath)
    $pllText = $pllText.Replace('defparam PLL_inst.IDIV_SEL = 1;', 'defparam PLL_inst.IDIV_SEL = 2;')
    $pllText = $pllText.Replace('defparam PLL_inst.MDIV_SEL = 7;', 'defparam PLL_inst.MDIV_SEL = 14;')
    [IO.File]::WriteAllText($outPll, $pllText, [Text.UTF8Encoding]::new($false))

    # Gowin writes version="1" in the XML declaration, which strict .NET XML
    # parsing rejects. Resolve each file path textually while preserving the
    # vendor file's exact structure.
    $projectText = [IO.File]::ReadAllText($projectPath)
    $pathPattern = 'path="([^"]+)"'
    $projectText = [regex]::Replace($projectText, $pathPattern, {
        param($match)
        $sourcePath = $match.Groups[1].Value
        $resolved = if ([IO.Path]::IsPathRooted($sourcePath)) {
            [IO.Path]::GetFullPath($sourcePath)
        } else {
            [IO.Path]::GetFullPath((Join-Path $projectDir $sourcePath))
        }
        $targetPath = if ($resolved -eq $pllPath) { $outPll } else { $resolved }
        'path="' + $targetPath.Replace('\', '/') + '"'
    })
    [IO.File]::WriteAllText($outProject, $projectText, [Text.UTF8Encoding]::new($false))

    [pscustomobject]@{
        name = $item.Name
        project = [IO.Path]::GetRelativePath($Root, $outProject).Replace('\', '/')
        top = $item.Top
        pll_source = [IO.Path]::GetRelativePath($Root, $pllPath).Replace('\', '/')
        pll_compat = [IO.Path]::GetRelativePath($Root, $outPll).Replace('\', '/')
        original_idiv = 1
        original_mdiv = 7
        compat_idiv = 2
        compat_mdiv = 14
        output_frequency_mhz = 125
    }
}

$manifestPath = Join-Path $Root 'reports/pll_compat_projects.csv'
$manifest | Export-Csv -NoTypeInformation -Encoding utf8 $manifestPath
$manifest | ConvertTo-Json -Depth 4 | Set-Content -Encoding utf8 (Join-Path $Root 'reports/pll_compat_projects.json')
$manifest | Format-Table -AutoSize
