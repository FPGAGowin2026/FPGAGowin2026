#requires -Version 7.3
[CmdletBinding()]
param([string]$GowinShell='D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe')
$ErrorActionPreference='Stop'
$ProjectDir=Split-Path -Parent $PSScriptRoot
$ProjectFile=Join-Path $ProjectDir 'choice3_robot_138k.gprj'
$TclFile=Join-Path $PSScriptRoot 'run_gowin.tcl'
$ReportDir=Join-Path $ProjectDir 'reports'
$LogFile=Join-Path $ReportDir 'gowin_syn.log'
$SummaryFile=Join-Path $ReportDir 'synthesis_summary.json'
$TargetPart='GW5AT-LV138PG484AC1/I0'
$TargetDevice='GW5AT-138B'
$TargetDatabase='gw5at138b-012'

if(!(Test-Path -LiteralPath $GowinShell)){throw "未找到 Gowin Shell：$GowinShell"}
if(!(Test-Path -LiteralPath $ProjectFile)){throw "缺少工程：$ProjectFile"}
if(!(Test-Path -LiteralPath $TclFile)){throw "缺少 Tcl：$TclFile"}
New-Item -ItemType Directory -Force -Path $ReportDir|Out-Null

# Strict preflight: synthesis only, with no board pin/timing claim.
[xml]$ProjectXml=Get-Content -LiteralPath $ProjectFile -Raw
$DeviceNode=$ProjectXml.Project.Device
if($DeviceNode.name-ne $TargetDevice-or $DeviceNode.pn-ne $TargetPart-or
   $DeviceNode.InnerText-ne $TargetDatabase){
  throw "目标器件不精确：name=$($DeviceNode.name) pn=$($DeviceNode.pn) db=$($DeviceNode.InnerText)"
}
$ConstraintEntries=@($ProjectXml.Project.FileList.File|Where-Object{
  $_.type -in @('file.cst','file.sdc') -or $_.path -match '\.(cst|sdc)$'})
if($ConstraintEntries.Count-ne 0){throw '综合工程不得包含未验证的 CST/SDC'}
$ConstraintDir=Join-Path $ProjectDir 'constraints'
$StaleConstraintFiles=@(Get-ChildItem -LiteralPath $ConstraintDir -File -ErrorAction SilentlyContinue|
  Where-Object{$_.Extension -in @('.cst','.sdc')})
if($StaleConstraintFiles.Count-ne 0){throw '138K 文件夹仍保留旧 CST/SDC'}
$TclText=Get-Content -LiteralPath $TclFile -Raw
if($TclText-notmatch '(?im)^\s*run\s+syn\s*$'){throw 'Tcl 未执行 run syn'}
if($TclText-match '(?im)^\s*run\s+pnr\s*$'){throw '禁止执行 PnR'}
if($TclText-notmatch 'choice3_robot_138k_top'){throw 'Tcl 顶层不是 choice3_robot_138k_top'}

Push-Location $ProjectDir
try{
  $Output=& $GowinShell $TclFile 2>&1
  $Code=$LASTEXITCODE
  $Output|Tee-Object -FilePath $LogFile
  if($Code-ne 0){throw "Gowin run syn 失败，退出码 $Code"}
}finally{Pop-Location}
$LogText=Get-Content -LiteralPath $LogFile -Raw
if($LogText-match '(?im)^\s*(error\s*:|fatal\s*:|ERROR\b|FATAL\b)'){
  throw 'Gowin 日志含错误/致命诊断'
}
if($LogText-match '(?i)place\s*(?:and|&)\s*route|\brun\s+pnr\b'){
  throw '日志表明意外执行了 PnR'
}
$WarningLines=@($LogText-split "`r?`n"|Where-Object{$_ -match '^WARN\s+\(([A-Z]+\d+)\)'})
$AllowedVendorCodes=@('EX3209','EX3801','EX2582','EX3791','EX3826','EX3827')
$UnexpectedWarnings=@($WarningLines|Where-Object{
  $_ -notmatch '[\\/]src[\\/]vendor[\\/]Gowin_PicoRV32[\\/]' -or
  (($_ -match '^WARN\s+\(([A-Z]+\d+)\)') -and $Matches[1] -notin $AllowedVendorCodes)
})
if($UnexpectedWarnings.Count-ne 0){
  throw "存在非白名单综合警告：$($UnexpectedWarnings-join ' | ')"
}

$Base='choice3_robot_138k'
$SynDir=Join-Path $ProjectDir 'impl\gwsynthesis'
$SynReport=Join-Path $SynDir "${Base}_syn.rpt.html"
$ResourceXml=Join-Path $SynDir "${Base}_syn_rsc.xml"
$Netlist=Join-Path $SynDir "${Base}.vg"
foreach($Path in @($SynReport,$ResourceXml,$Netlist)){
  if(!(Test-Path -LiteralPath $Path)){throw "缺少综合交付文件：$Path"}
  if((Get-Item -LiteralPath $Path).LastWriteTime-lt (Get-Item -LiteralPath $LogFile).LastWriteTime.AddMinutes(-10)){
    throw "综合文件疑似陈旧：$Path"
  }
}
$ReportText=Get-Content -LiteralPath $SynReport -Raw
if($ReportText-notmatch ('(?s)Part Number</td>\s*<td>'+[regex]::Escape($TargetPart)+'</td>')){
  throw '综合报告 Part Number 校验失败'
}
if($ReportText-notmatch '(?s)Device</td>\s*<td>GW5AT-138</td>'-or
   $ReportText-notmatch '(?s)Device Version</td>\s*<td>B</td>'){
  throw '综合报告 Device/Version 校验失败'
}

[xml]$ResourceDoc=Get-Content -LiteralPath $ResourceXml -Raw
$Root=$ResourceDoc.Module
if($Root.name-ne 'choice3_robot_138k_top'){throw "资源报告顶层错误：$($Root.name)"}
$OsdNode=$ResourceDoc.SelectSingleNode("//SubModule[@name='u_attitude_osd']")
if($null-eq $OsdNode){throw '姿态 OSD 未保留在综合资源层级，拒绝交付'}
function Get-ResourceTotal([object]$Node,[string]$Name){
  $Value=[string]$Node.GetAttribute($Name)
  if([string]::IsNullOrWhiteSpace($Value)){return 0}
  if($Value-match '^(\d+)'){return [int]$Matches[1]}
  throw "无法解析资源属性 $Name=$Value"
}
$Register=Get-ResourceTotal $Root 'T_Register'
$Lut=Get-ResourceTotal $Root 'T_Lut'
$Alu=Get-ResourceTotal $Root 'T_Alu'
$Bsram=Get-ResourceTotal $Root 'T_Bsram'
$Mult18=Get-ResourceTotal $Root 'T_MULTALU27X18'
$Mult36=Get-ResourceTotal $Root 'T_MULT27X36'
$Logic=$Lut+$Alu
$LogicCapacity=0;$RegisterCapacity=0;$BsramCapacity=0
if($ReportText-match '(?s)<td class="label">Logic</td>\s*<td>\s*\d+\s*\([^<]*\)\s*/\s*(\d+)\s*</td>'){$LogicCapacity=[int]$Matches[1]}
if($ReportText-match '(?s)<td class="label">Register</td>\s*<td>\s*\d+\s*/\s*(\d+)\s*</td>'){$RegisterCapacity=[int]$Matches[1]}
if($ReportText-match '(?s)<td class="label">BSRAM</td>\s*<td>\s*\d+\s*/\s*(\d+)\s*</td>'){$BsramCapacity=[int]$Matches[1]}
if($LogicCapacity-eq 0-or $RegisterCapacity-eq 0-or $BsramCapacity-eq 0){
  throw '无法从综合报告解析器件资源容量'
}
$Summary=[ordered]@{
  generated_at=(Get-Date -Format o);flow='Gowin run syn only';top='choice3_robot_138k_top'
  device=$TargetDevice;part=$TargetPart;database=$TargetDatabase
  logic_used=$Logic;logic_capacity=$LogicCapacity;lut_used=$Lut;alu_used=$Alu
  registers_used=$Register;register_capacity=$RegisterCapacity
  bsram_used=$Bsram;bsram_capacity=$BsramCapacity
  multalu27x18_used=$Mult18;mult27x36_used=$Mult36
  warning_count=$WarningLines.Count;warning_scope='official Gowin PicoRV32 vendor source only'
  attitude_osd_preserved=$true
  report=$SynReport;resource_xml=$ResourceXml;netlist=$Netlist;log=$LogFile
  pnr_run=$false
}
$Summary|ConvertTo-Json -Depth 3|Set-Content -LiteralPath $SummaryFile -Encoding utf8
Write-Host "PASS: Gowin synthesis-only $TargetPart"
Write-Host "RESOURCE: Logic=$Logic/$LogicCapacity (LUT=$Lut ALU=$Alu) Register=$Register/$RegisterCapacity BSRAM=$Bsram/$BsramCapacity MULTALU27X18=$Mult18 MULT27X36=$Mult36"
Write-Host "REPORT: $SynReport"
Write-Host 'PNR: NOT RUN'
