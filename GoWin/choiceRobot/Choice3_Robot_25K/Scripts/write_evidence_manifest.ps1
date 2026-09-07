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
foreach($dir in @('src','sim','constraints','Scripts')){
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
foreach($base in @('choice3_robot_25k','choice3_robot_25k_demo')){
  foreach($relative in @(
    "impl/gwsynthesis/$base.log",
    "impl/gwsynthesis/$($base)_syn.rpt.html",
    "impl/gwsynthesis/$($base)_syn_rsc.xml",
    "impl/pnr/$base.rpt.txt",
    "impl/pnr/$($base)_tr_content.html",
    "impl/pnr/$base.fs"
  )){
    $path=Join-Path $projectDir $relative
    if(!(Test-Path -LiteralPath $path)){throw "缺少证据文件：$path"}
    $evidenceFiles+=Get-Item -LiteralPath $path
  }
}
$evidenceFiles=$evidenceFiles|Sort-Object FullName -Unique
$manifest=[ordered]@{
  schema='choice3-evidence-manifest-v1'
  generated_at=(Get-Date).ToUniversalTime().ToString('o')
  project='Choice3_Robot_25K'
  target='GW5A-LV25MG121NC1/I0'
  scope='Hashes bind current source, tests, constraints, scripts and firmware HEX to retained simulation, synthesis, PnR, bitstream, programmer and UART evidence.'
  inputs=@($inputFiles|ForEach-Object{New-HashEntry $_})
  evidence=@($evidenceFiles|ForEach-Object{New-HashEntry $_})
}
New-Item -ItemType Directory -Force -Path $reportDir|Out-Null
[IO.File]::WriteAllText($outputPath,($manifest|ConvertTo-Json -Depth 6)+"`n",[Text.UTF8Encoding]::new($false))
Write-Host "EVIDENCE MANIFEST PASS：$outputPath"
