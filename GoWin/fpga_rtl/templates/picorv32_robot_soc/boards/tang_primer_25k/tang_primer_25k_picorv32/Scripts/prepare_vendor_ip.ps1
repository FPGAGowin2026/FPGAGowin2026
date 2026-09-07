param(
    [string]$GowinIpDir = "D:\Gowin\Gowin_V1.9.12.03_x64\IDE\ipcore\GowinPicoRV32\Gowin_PicoRV32",
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectDir = [System.IO.Path]::GetFullPath((Join-Path $scriptDir ".."))
$templateDir = [System.IO.Path]::GetFullPath((Join-Path $projectDir "..\..\.."))
$firmwareBuilder = Join-Path $templateDir "common\firmware\build_firmware.py"
$firmwareBuildDir = Join-Path $templateDir "common\firmware\build"
$vendorDir = Join-Path $projectDir "vendor_ip\Gowin_PicoRV32"
$firmwareDir = Join-Path $projectDir "firmware"
$configSource = Join-Path $projectDir "config\pico_define_25k.vh"

if (-not (Test-Path -LiteralPath $GowinIpDir -PathType Container)) {
    throw "未找到官方 Gowin PicoRV32 RTL：$GowinIpDir"
}
if (-not (Test-Path -LiteralPath $firmwareBuilder -PathType Leaf)) {
    throw "未找到固件生成器：$firmwareBuilder"
}

& $Python $firmwareBuilder --out-dir $firmwareBuildDir
if ($LASTEXITCODE -ne 0) {
    throw "自检固件生成失败，退出码：$LASTEXITCODE"
}

New-Item -ItemType Directory -Force -Path $vendorDir, $firmwareDir | Out-Null

$requiredFiles = @(
    "picosoc.v",
    "picorv32.v",
    "picosoc_itcm.v",
    "picosoc_dtcm.v",
    "simpleuart.v",
    "wb_bus.v",
    "wb_brancher.v",
    "pico_config.vh",
    "pico_csr_define.vh",
    "advspi_config.vh",
    "advspi_const.vh"
)

foreach ($name in $requiredFiles) {
    $source = Join-Path $GowinIpDir $name
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
        throw "官方 IP 文件缺失：$source"
    }
    Copy-Item -LiteralPath $source -Destination (Join-Path $vendorDir $name) -Force
}

# Gowin 1.9.12.03 安装包中的 picosoc.v 把两个组合信号声明成了 reg，
# 同一文件稍后却用连续 assign 驱动；GowinSynthesis 会报 EX3805。
# 官方 IP 生成器通常会在输出阶段修正模板，这里对本地准备副本做同样的确定性修正。
$picoSocPath = Join-Path $vendorDir "picosoc.v"
$picoSocText = [System.IO.File]::ReadAllText($picoSocPath)
$oldReady = "    reg         mem_ready;"
$oldData = "    reg  [31:0] mem_rdata;"
if (-not $picoSocText.Contains($oldReady) -or -not $picoSocText.Contains($oldData)) {
    throw "picosoc.v 模板与已验证的 Gowin 1.9.12.03 版本不一致，拒绝盲目修改。"
}
$picoSocText = $picoSocText.Replace($oldReady, "    wire        mem_ready;")
$picoSocText = $picoSocText.Replace($oldData, "    wire [31:0] mem_rdata;")

# 官方模板内部复位移位寄存器使用下降沿，却直接释放给上升沿 CPU，
# 50 MHz 下只留下 10 ns 半周期并形成真实 setup 违例。改为同域上升沿后，
# 非阻塞赋值仍保证 CPU 到下一拍才看到复位释放，且保留原有 16 拍延时。
$oldResetEdge = "    always @(negedge clk_in)"
if (-not $picoSocText.Contains($oldResetEdge)) {
    throw "picosoc.v 未找到已验证的内部复位下降沿模板。"
}
$picoSocText = $picoSocText.Replace($oldResetEdge, "    always @(posedge clk_in)")
[System.IO.File]::WriteAllText(
    $picoSocPath,
    $picoSocText,
    [System.Text.UTF8Encoding]::new($false)
)

Copy-Item -LiteralPath $configSource -Destination (Join-Path $vendorDir "pico_define.vh") -Force
Copy-Item -LiteralPath (Join-Path $firmwareBuildDir "picorv32_selftest.hex") -Destination (Join-Path $firmwareDir "picorv32_selftest.hex") -Force
Copy-Item -LiteralPath (Join-Path $firmwareBuildDir "picorv32_selftest.json") -Destination (Join-Path $firmwareDir "picorv32_selftest.json") -Force

Write-Host "已准备 Gowin PicoRV32 官方 RTL：$vendorDir"
Write-Host "已准备 ITCM 固件：$firmwareDir\picorv32_selftest.hex"
