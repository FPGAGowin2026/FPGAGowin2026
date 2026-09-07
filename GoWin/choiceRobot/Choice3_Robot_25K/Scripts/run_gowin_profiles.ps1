#requires -Version 7.3
[CmdletBinding()]
param(
    [ValidateSet('Release','Demo','All')][string]$Profile='All',
    [string]$GowinShell='D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'
)
$ErrorActionPreference='Stop'
$projectDir=Split-Path -Parent $PSScriptRoot
$reportDir=Join-Path $projectDir 'reports'
if(!(Test-Path -LiteralPath $GowinShell)){throw "未找到 Gowin Shell：$GowinShell"}
New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
$profiles=if($Profile-eq'All'){@('Release','Demo')}else{@($Profile)}
foreach($item in $profiles){
    $isDemo=$item-eq'Demo'
    $base=if($isDemo){'choice3_robot_25k_demo'}else{'choice3_robot_25k'}
    $tclName=if($isDemo){'run_gowin_demo.tcl'}else{'run_gowin_release.tcl'}
    $tcl=Join-Path $PSScriptRoot $tclName
    Push-Location $projectDir
    try{
        $output=& $GowinShell $tcl 2>&1
        $code=$LASTEXITCODE
        $output|Tee-Object -FilePath (Join-Path $reportDir ($base+'.gowin.log'))
        if($code-ne 0){throw "$item Gowin 构建失败，退出码 $code"}
    }finally{Pop-Location}
    $synthLog=Join-Path $projectDir "impl\gwsynthesis\$base.log"
    if(!(Test-Path -LiteralPath $synthLog)){throw "缺少综合日志：$synthLog"}
    $synthText=Get-Content -LiteralPath $synthLog -Raw
    if(!$isDemo){
        if($synthText-match '(?i)choice3_attitude_osd[^\r\n]*swept'){
            throw 'Release OSD 被综合优化删除'
        }
        $userTruncation=@(Select-String -LiteralPath $synthLog -Pattern '^WARN\s+\(EX3791\)' |
          Where-Object {$_.Line -notmatch '\\src\\vendor\\Gowin_PicoRV32\\'})
        if($userTruncation.Count-ne 0){throw "Release 用户 RTL 存在 EX3791 截断告警：$($userTruncation.Line -join '; ')"}
        $resourceXml=Join-Path $projectDir "impl\gwsynthesis\${base}_syn_rsc.xml"
        if(!(Test-Path -LiteralPath $resourceXml) -or
           (Get-Content -LiteralPath $resourceXml -Raw)-notmatch '<SubModule name="u_attitude_osd"'){
            throw 'Release 综合资源报告未保留 u_attitude_osd 层级'
        }
    }
    $bitstream=Join-Path $projectDir "impl\pnr\$base.fs"
    $pnr=Join-Path $projectDir "impl\pnr\$base.rpt.txt"
    $timing=Join-Path $projectDir "impl\pnr\${base}_tr_content.html"
    foreach($path in @($bitstream,$pnr,$timing)){if(!(Test-Path -LiteralPath $path)){throw "缺少交付文件：$path"}}
    $pnrText=Get-Content -LiteralPath $pnr -Raw
    $timingText=Get-Content -LiteralPath $timing -Raw
    if($pnrText-notmatch '<Part Number>: GW5A-LV25MG121NC1/I0' -or $pnrText-notmatch '<Device Version>: B'){
        throw "$item 目标器件或版本不正确"
    }
    if($timingText-notmatch 'Numbers of Setup Violated Endpoints</td>\s*<td>0</td>' -or
       $timingText-notmatch 'Numbers of Hold Violated Endpoints</td>\s*<td>0</td>'){
        throw "$item STA 存在 setup/hold 违规"
    }
    if($pnrText-match '(?m)^\s*Logic\s+\|\s*(\d+)/23040'){$logic=[int]$Matches[1]}else{throw '无法解析 Logic'}
    if($pnrText-match '(?m)^\s*BSRAM\s+\|\s*(\d+)/56'){$bsram=[int]$Matches[1]}else{throw '无法解析 BSRAM'}
    if($pnrText-match '(?m)^\s*DSP\s+\|\s*(\d+)/28'){$dsp=[int]$Matches[1]}else{throw '无法解析 DSP'}
    if($logic-gt 18000-or $bsram-gt 52-or $dsp-gt 24){throw "$item 资源越界 Logic=$logic BSRAM=$bsram DSP=$dsp"}
    $hash=(Get-FileHash -Algorithm SHA256 -LiteralPath $bitstream).Hash.ToLowerInvariant()
    Write-Host "PASS $item：Logic=$logic BSRAM=$bsram DSP=$dsp SHA256=$hash"
}
