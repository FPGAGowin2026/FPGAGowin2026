param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$projectPath = Join-Path $Root 'raw/ch54/ch54_udp_send_rgmii/udp_send_rgmii_with_checksum/udp_send_rgmii/udp_send_rgmii.gprj'
$projectPath = [IO.Path]::GetFullPath($projectPath)
$projectDir = Split-Path -Parent $projectPath
$outDir = Join-Path $Root 'normalized/ch54_udp_send_config_pin_compat'
$outProject = Join-Path $outDir 'udp_send_rgmii.gprj'

New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$projectText = [IO.File]::ReadAllText($projectPath)
$projectText = [regex]::Replace($projectText, 'path="([^"]+)"', {
    param($match)
    $sourcePath = $match.Groups[1].Value
    $resolved = if ([IO.Path]::IsPathRooted($sourcePath)) {
        [IO.Path]::GetFullPath($sourcePath)
    } else {
        [IO.Path]::GetFullPath((Join-Path $projectDir $sourcePath))
    }
    'path="' + $resolved.Replace('\', '/') + '"'
})
[IO.File]::WriteAllText($outProject, $projectText, [Text.UTF8Encoding]::new($false))

[pscustomobject]@{
    project = [IO.Path]::GetRelativePath($Root, $outProject).Replace('\', '/')
    top = 'eth_send_test_rgmii'
    compatibility = 'reuse ACG720 CPU/SSPI configuration pins as regular GPIO'
}
