#requires -Version 7.3
[CmdletBinding()]
param([string]$GowinShell='D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe')
$ErrorActionPreference='Stop'
$ProjectDir=Split-Path -Parent $PSScriptRoot
$TclFile=Join-Path $PSScriptRoot 'run_gowin.tcl'
$ReportDir=Join-Path $ProjectDir 'reports'
$LogFile=Join-Path $ReportDir 'gowin.log'
if(!(Test-Path -LiteralPath $GowinShell)){throw "未找到 Gowin Shell：$GowinShell"}
New-Item -ItemType Directory -Force $ReportDir | Out-Null
Push-Location $ProjectDir
try{
  $Output=& $GowinShell $TclFile 2>&1
  $Code=$LASTEXITCODE
  $Output|Tee-Object -FilePath $LogFile
  if($Code-ne 0){throw "Gowin 构建失败，退出码 $Code"}
}finally{Pop-Location}

$Base='tang_primer_25k_choice3_robot'
$Bitstream=Join-Path $ProjectDir "impl\pnr\$Base.fs"
$PnrReport=Join-Path $ProjectDir "impl\pnr\$Base.rpt.txt"
$TimingReport=Join-Path $ProjectDir "impl\pnr\${Base}_tr_content.html"
foreach($Path in @($Bitstream,$PnrReport,$TimingReport)){
  if(!(Test-Path -LiteralPath $Path)){throw "Gowin 缺少交付文件：$Path"}
}
$PnrText=Get-Content -LiteralPath $PnrReport -Raw
$TimingText=Get-Content -LiteralPath $TimingReport -Raw
if($PnrText-notmatch '<Part Number>: GW5A-LV25MG121NC1/I0' -or
   $PnrText-notmatch '<Device Version>: B'){throw '目标器件不是 GW5A-LV25MG121NC1/I0 Device B'}
if($TimingText-notmatch 'Numbers of Setup Violated Endpoints</td>\s*<td>0</td>' -or
   $TimingText-notmatch 'Numbers of Hold Violated Endpoints</td>\s*<td>0</td>'){
  throw 'STA 存在 setup/hold 违规或报告格式无法确认'
}
if($PnrText-match '(?m)^\s*Logic\s+\|\s*(\d+)/23040'){$Logic=[int]$Matches[1]}else{throw '无法解析 Logic 资源'}
if($PnrText-match '(?m)^\s*BSRAM\s+\|\s*(\d+)/56'){$Bsram=[int]$Matches[1]}else{throw '无法解析 BSRAM 资源'}
if($PnrText-match '(?m)^\s*DSP\s+\|\s*(\d+)/28'){$Dsp=[int]$Matches[1]}else{throw '无法解析 DSP 资源'}
if($Logic-gt 18000-or $Bsram-gt 44-or $Dsp-gt 20){
  throw "资源越过审计边界：Logic=$Logic/18000 BSRAM=$Bsram/44 DSP=$Dsp/20"
}
$Hash=(Get-FileHash -Algorithm SHA256 -LiteralPath $Bitstream).Hash.ToLowerInvariant()
$Size=(Get-Item -LiteralPath $Bitstream).Length
Write-Host "GOWIN BUILD PASS：$Bitstream"
Write-Host "RESOURCE GATE PASS：Logic=$Logic BSRAM=$Bsram DSP=$Dsp"
Write-Host "BITSTREAM：bytes=$Size sha256=$Hash"
Write-Host '说明：脚本只构建，不调用 programmer，不会烧录板卡。'
