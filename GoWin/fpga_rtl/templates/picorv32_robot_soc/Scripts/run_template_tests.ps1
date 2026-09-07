[CmdletBinding()]
param(
    [string]$Python = "python",
    [string]$Icarus = "$env:LOCALAPPDATA\Programs\IcarusVerilog\bin\iverilog.exe",
    [string]$Vvp = "$env:LOCALAPPDATA\Programs\IcarusVerilog\bin\vvp.exe"
)

$ErrorActionPreference = "Stop"
$templateRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$testOutput = Join-Path $env:TEMP "gowin_picorv32_template_tests"

if (-not (Get-Command $Python -ErrorAction SilentlyContinue)) {
    throw "未找到 Python：$Python"
}
if (-not (Test-Path -LiteralPath $Icarus)) {
    throw "未找到 Icarus Verilog 编译器：$Icarus"
}
if (-not (Test-Path -LiteralPath $Vvp)) {
    throw "未找到 Icarus Verilog 运行器：$Vvp"
}

New-Item -ItemType Directory -Force -Path $testOutput | Out-Null
Push-Location $templateRoot

try {
    & $Python ".\common\firmware\build_firmware.py"
    if ($LASTEXITCODE -ne 0) {
        throw "固件生成或 RV32I 解释执行自检失败。"
    }

    $serviceRegsImage = Join-Path $testOutput "robot_service_regs.vvp"
    & $Icarus -g2012 -Wall -s tb_robot_service_regs -o $serviceRegsImage `
        ".\common\rtl\robot_service_regs.v" `
        ".\common\sim\tb_robot_service_regs.sv"
    if ($LASTEXITCODE -ne 0) {
        throw "robot_service_regs 编译失败。"
    }
    & $Vvp $serviceRegsImage
    if ($LASTEXITCODE -ne 0) {
        throw "robot_service_regs 回归失败。"
    }

    $watchdogImage = Join-Path $testOutput "softcore_service_watchdog.vvp"
    & $Icarus -g2012 -Wall -s tb_softcore_service_watchdog -o $watchdogImage `
        ".\common\rtl\softcore_service_watchdog.v" `
        ".\common\sim\tb_softcore_service_watchdog.sv"
    if ($LASTEXITCODE -ne 0) {
        throw "softcore_service_watchdog 编译失败。"
    }
    & $Vvp $watchdogImage
    if ($LASTEXITCODE -ne 0) {
        throw "softcore_service_watchdog 回归失败。"
    }

    Write-Host "PASS：固件解释执行、Wishbone 寄存器组与服务核看门狗全部通过。"
}
finally {
    Pop-Location
}
