#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Programmer='D:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe',
    [int]$CableIndex=4
)
$ErrorActionPreference='Stop'
$projectDir=Split-Path -Parent $PSScriptRoot
$bitstream=Join-Path $projectDir 'impl\pnr\choice3_robot_25k_demo.fs'
$reportDir=Join-Path $projectDir 'reports'
$log=Join-Path $reportDir 'board_program_demo.log'
if(!(Test-Path -LiteralPath $Programmer)){throw "未找到 Programmer：$Programmer"}
if(!(Test-Path -LiteralPath $bitstream)){throw "未生成 DEMO 位流：$bitstream"}
New-Item -ItemType Directory -Force -Path $reportDir|Out-Null
$hash=(Get-FileHash -Algorithm SHA256 -LiteralPath $bitstream).Hash.ToLowerInvariant()
@(
  "Choice3 25K SRAM programming $(Get-Date -Format o)",
  "PROGRAMMER=$Programmer",
  "CABLE_INDEX=$CableIndex",
  "BITSTREAM=$bitstream",
  "BITSTREAM_SHA256=$hash",
  '--- DEVICE SCAN ---'
)|Set-Content -LiteralPath $log
$scanText=(& $Programmer --cable-index $CableIndex --scan 2>&1|Tee-Object -FilePath $log -Append|Out-String)
$scanCode=$LASTEXITCODE
if($scanCode-ne 0){throw "器件扫描失败，退出码 $scanCode"}
if($scanText-notmatch '(?i)0x0001281B'){throw '扫描结果未发现预期器件 ID 0x0001281B，拒绝下载'}
'--- SRAM PROGRAM ---'|Tee-Object -FilePath $log -Append
$programText=(& $Programmer --device GW5A-25B --operation_index 2 --fsFile $bitstream --cable-index $CableIndex 2>&1|Tee-Object -FilePath $log -Append|Out-String)
$programCode=$LASTEXITCODE
if($programCode-ne 0){throw "SRAM 下载失败，退出码 $programCode"}
if($programText-notmatch '(?im)^\s*Finished\.\s*$' -or
   $programText-notmatch '(?im)^\s*User Code is:\s*0x[0-9A-F]+\s*$' -or
   $programText-notmatch '(?im)^\s*Status Code is:\s*0x[0-9A-F]+\s*$'){
  throw 'Programmer 未同时输出 Finished/User Code/Status Code，拒绝判定为 PASS'
}
"SRAM PROGRAM PASS：device_id=0x0001281B sha256=$hash"|Tee-Object -FilePath $log -Append
Write-Host "SRAM PROGRAM PASS：$bitstream"
