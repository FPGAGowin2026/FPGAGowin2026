#requires -Version 7.3
[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$projectDir=Split-Path -Parent $PSScriptRoot
$reportDir=Join-Path $projectDir 'reports'
$outputPath=Join-Path $reportDir 'evidence_manifest.json'

function New-HashEntry([System.IO.FileInfo]$file){
  [ordered]@{
    path=([IO.Path]::GetRelativePath($projectDir,$file.FullName)-replace '\\','/')
    bytes=$file.Length
    sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $file.FullName).Hash.ToLowerInvariant()
    last_write_utc=$file.LastWriteTimeUtc.ToString('o')
  }
}
$inputFiles=@()
foreach($dir in @('src','sim','Scripts')){
  $inputFiles+=Get-ChildItem -LiteralPath (Join-Path $projectDir $dir) -Recurse -File
}
$inputFiles+=Get-ChildItem -LiteralPath $projectDir -File|Where-Object{$_.Extension-in @('.gprj','.md')}
$inputFiles+=Get-Item -LiteralPath (Join-Path $projectDir 'firmware/build_robot_firmware.py')
$inputFiles+=Get-Item -LiteralPath (Join-Path $projectDir 'firmware/picorv32_selftest.hex')
$inputFiles=$inputFiles|Sort-Object FullName -Unique

$evidenceFiles=@()
$evidenceFiles+=Get-ChildItem -LiteralPath $reportDir -File|Where-Object{$_.Name-ne'evidence_manifest.json'}
$evidenceFiles+=Get-ChildItem -LiteralPath (Join-Path $projectDir 'firmware') -File|
  Where-Object{$_.Name-in @('picorv32_selftest.json','picorv32_selftest.lst')}
foreach($relative in @(
  'impl/gwsynthesis/choice3_robot_138k.log',
  'impl/gwsynthesis/choice3_robot_138k_syn.rpt.html',
  'impl/gwsynthesis/choice3_robot_138k_syn_rsc.xml',
  'impl/gwsynthesis/choice3_robot_138k.vg'
)){
  $path=Join-Path $projectDir $relative
  if(!(Test-Path -LiteralPath $path)){throw "缺少证据文件：$path"}
  $evidenceFiles+=Get-Item -LiteralPath $path
}
$evidenceFiles=$evidenceFiles|Sort-Object FullName -Unique
$manifest=[ordered]@{
  schema='choice3-evidence-manifest-v1'
  generated_at=(Get-Date).ToUniversalTime().ToString('o')
  project='Choice3_Robot_138K'
  target='GW5AT-LV138PG484AC1/I0'
  scope='Hashes bind current source, tests, scripts and firmware HEX to retained simulation, synthesis reports and synthesized netlist. No PnR evidence is claimed.'
  inputs=@($inputFiles|ForEach-Object{New-HashEntry $_})
  evidence=@($evidenceFiles|ForEach-Object{New-HashEntry $_})
}
New-Item -ItemType Directory -Force -Path $reportDir|Out-Null
[IO.File]::WriteAllText($outputPath,($manifest|ConvertTo-Json -Depth 6)+"`n",[Text.UTF8Encoding]::new($false))
Write-Host "EVIDENCE MANIFEST PASS：$outputPath"
