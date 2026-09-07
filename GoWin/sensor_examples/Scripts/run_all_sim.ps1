#requires -Version 7.3
[CmdletBinding()]
param(
    [string]$Iverilog = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe',
    [string]$Vvp = 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe'
)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$buildRoot = Join-Path $root 'build\sim'
New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null

if (-not (Test-Path -LiteralPath $Iverilog -PathType Leaf)) {
    throw "未找到 Icarus Verilog 编译器：$Iverilog"
}
if (-not (Test-Path -LiteralPath $Vvp -PathType Leaf)) {
    throw "未找到 Icarus Verilog 运行器：$Vvp"
}

$projects = Get-ChildItem -LiteralPath $root -Directory |
    Where-Object { Test-Path -LiteralPath (Join-Path $_.FullName 'sensor_example.gprj') } |
    Sort-Object Name

$failures = [System.Collections.Generic.List[string]]::new()
foreach ($project in $projects) {
    [xml]$projectXml = Get-Content -LiteralPath (Join-Path $project.FullName 'sensor_example.gprj') -Raw
    $rtlFiles = @($projectXml.Project.FileList.File |
        Where-Object { $_.type -eq 'file.verilog' -and $_.enable -eq '1' } |
        ForEach-Object { Join-Path $project.FullName $_.path })
    $testbench = Get-ChildItem -LiteralPath (Join-Path $project.FullName 'sim') -Filter 'tb_*.sv' |
        Sort-Object Name | Select-Object -First 1
    $reportDir = Join-Path $project.FullName 'reports'
    $logFile = Join-Path $reportDir 'simulation.log'
    New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
    Set-Content -LiteralPath $logFile -Encoding UTF8 -Value "[$($project.Name)] RTL 编译与自检仿真"

    if (($rtlFiles.Count -lt 1) -or (-not $testbench) -or
        ($rtlFiles | Where-Object { -not (Test-Path -LiteralPath $_ -PathType Leaf) })) {
        $failures.Add("$($project.Name): 工程结构不完整")
        Add-Content -LiteralPath $logFile -Encoding UTF8 -Value 'FAIL: 工程结构不完整'
        continue
    }

    $topBuild = Join-Path $buildRoot ($project.Name + '_rtl.vvp')
    $compileOutput = & $Iverilog -g2001 -Wall -s sensor_example_top -o $topBuild @rtlFiles 2>&1
    $compileCode = $LASTEXITCODE
    $compileOutput | Add-Content -LiteralPath $logFile -Encoding UTF8
    if ($compileCode -ne 0) {
        $failures.Add("$($project.Name): Verilog-2001 RTL 编译失败")
        continue
    }

    $moduleMatch = Select-String -LiteralPath $testbench.FullName -Pattern '^\s*module\s+(tb_[A-Za-z0-9_]+)' |
        Select-Object -First 1
    if (-not $moduleMatch) {
        $failures.Add("$($project.Name): 无法识别 testbench 顶层")
        continue
    }
    $testbenchTop = $moduleMatch.Matches[0].Groups[1].Value
    $simBuild = Join-Path $buildRoot ($project.Name + '_tb.vvp')
    $tbCompileOutput = & $Iverilog -g2012 -Wall -s $testbenchTop -o $simBuild @rtlFiles $testbench.FullName 2>&1
    $tbCompileCode = $LASTEXITCODE
    $tbCompileOutput | Add-Content -LiteralPath $logFile -Encoding UTF8
    if ($tbCompileCode -ne 0) {
        $failures.Add("$($project.Name): testbench 编译失败")
        continue
    }

    $runOutput = & $Vvp $simBuild 2>&1
    $runCode = $LASTEXITCODE
    $runOutput | Tee-Object -FilePath $logFile -Append
    $passLines = @($runOutput | Where-Object { $_ -match '^PASS:' })
    $failureLines = @($runOutput | Where-Object { $_ -match '^(FAIL|FATAL|ERROR):' })
    if ($runCode -ne 0 -or $passLines.Count -ne 1 -or $failureLines.Count -ne 0) {
        $failures.Add("$($project.Name): 自检仿真失败")
        continue
    }
    Write-Host "PASS $($project.Name)" -ForegroundColor Green
}

if ($projects.Count -eq 0) {
    throw '未发现任何 sensor_example.gprj。'
}
if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Host "FAIL $_" -ForegroundColor Red }
    exit 1
}
Write-Host "全部 $($projects.Count) 个传感器工程的 RTL 编译与自检仿真通过。" -ForegroundColor Green
