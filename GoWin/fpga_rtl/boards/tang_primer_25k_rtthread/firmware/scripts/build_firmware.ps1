param(
    [string]$RtThreadRoot = "D:\RT-ThreadStudio\repo\Extract\RT-Thread_Source_Code\RT-Thread\4.1.1",
    [string]$ToolchainBin = "D:\RT-ThreadStudio\repo\Extract\ToolChain_Support_Packages\KENDRYTE\RISC-V-GCC-KENDRYTE\8.3.0\bin",
    [ValidateSet("xip")]
    [string]$Mode = "xip"
)

$ErrorActionPreference = "Stop"
$firmwareRoot = Split-Path -Parent $PSScriptRoot
$boardRoot = Split-Path -Parent $firmwareRoot
$buildDir = Join-Path $firmwareRoot "build\$Mode"
$ipRoot = Join-Path $boardRoot "tang_primer_25k_rtthread_ipgen\src\gowin_gorv32"
$gorv32Config = Join-Path $ipRoot "temp\gowin_gorv32\gorv32_config.vh"
$gorv32Ipc = Join-Path $ipRoot "gowin_gorv32.ipc"

$gcc = Join-Path $ToolchainBin "riscv-none-embed-gcc.exe"
$objcopy = Join-Path $ToolchainBin "riscv-none-embed-objcopy.exe"
$objdump = Join-Path $ToolchainBin "riscv-none-embed-objdump.exe"
$readelf = Join-Path $ToolchainBin "riscv-none-embed-readelf.exe"
$nm = Join-Path $ToolchainBin "riscv-none-embed-nm.exe"
$size = Join-Path $ToolchainBin "riscv-none-embed-size.exe"

foreach ($required in @($gcc, $objcopy, $objdump, $readelf, $nm, $size,
                         $gorv32Config, $gorv32Ipc,
                         (Join-Path $RtThreadRoot "src\thread.c"),
                         (Join-Path $RtThreadRoot "include\rtdef.h")))
{
    if (-not (Test-Path -LiteralPath $required))
    {
        throw "缺少构建依赖：$required"
    }
}

# 本 BSP 按 RT-Thread 4.1.1 的内核与 FinSH 接口审计，拒绝静默替换版本。
$rtdefText = Get-Content -LiteralPath (Join-Path $RtThreadRoot "include\rtdef.h") -Raw
if ($rtdefText -notmatch '#define\s+RT_VERSION\s+4\b' -or
    $rtdefText -notmatch '#define\s+RT_SUBVERSION\s+1\b' -or
    $rtdefText -notmatch '#define\s+RT_REVISION\s+1\b')
{
    throw "RT-Thread 源码版本不是已审计的 4.1.1：$RtThreadRoot"
}

# 固件的链接布局依赖这三项 IP 配置；在编译前拒绝静默使用回落默认值。
$configText = Get-Content -LiteralPath $gorv32Config -Raw
$ipcText = Get-Content -LiteralPath $gorv32Ipc -Raw
foreach ($requiredConfig in @(
    '`define ONCHIP_RAM_SIZE 14',
    "`define FLASH_BURN_ADDR 24'h100000",
    '`define UART1_SUPPORT'))
{
    if (-not $configText.Contains($requiredConfig))
    {
        throw "GoRV32 IP 配置不匹配：缺少 $requiredConfig"
    }
}
if ($configText.Contains('`define UART2_SUPPORT'))
{
    throw "GoRV32 IP 配置不匹配：不应启用 UART2"
}
foreach ($requiredIpc in @(
    'Flash_Burn_Address=100000',
    'Enable_UART1=true',
    'Enable_UART2=false',
    'SRAM_Size=64KB'))
{
    if (-not $ipcText.Contains($requiredIpc))
    {
        throw "GoRV32 IPC 配置不匹配：缺少 $requiredIpc"
    }
}
Write-Host "GoRV32 IP 配置核验通过：64KB SRAM、UART1、Flash 0x100000"

New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$objectDir = Join-Path $buildDir "obj"
New-Item -ItemType Directory -Force -Path $objectDir | Out-Null

$includes = @(
    (Join-Path $firmwareRoot "include"),
    (Join-Path $RtThreadRoot "include"),
    (Join-Path $RtThreadRoot "libcpu\risc-v\common"),
    (Join-Path $RtThreadRoot "components\finsh")
)

$commonFlags = @(
    "-march=rv32imc", "-mabi=ilp32", "-mcmodel=medany", "-mstrict-align",
    "-Os", "-g3", "-ffunction-sections", "-fdata-sections", "-fno-common",
    # 本工具链默认 char 为 unsigned；FinSH 用 char 的 -1 表示“暂无输入”。
    "-fsigned-char", "-fno-builtin", "-fno-strict-aliasing", "-Wall", "-Wextra",
    "-Wno-unused-parameter", "-D__RTTHREAD__"
)
$includeFlags = @()
foreach ($path in $includes)
{
    $includeFlags += "-I$path"
}

$sources = @(
    (Join-Path $firmwareRoot "startup\crt.S"),
    (Join-Path $firmwareRoot "bsp\board.c"),
    (Join-Path $firmwareRoot "bsp\trap.c"),
    (Join-Path $firmwareRoot "bsp\interrupt_gcc.S"),
    (Join-Path $firmwareRoot "app\main.c"),
    (Join-Path $RtThreadRoot "libcpu\risc-v\common\cpuport.c"),
    (Join-Path $RtThreadRoot "libcpu\risc-v\common\context_gcc.S"),
    (Join-Path $RtThreadRoot "src\clock.c"),
    (Join-Path $RtThreadRoot "src\components.c"),
    (Join-Path $RtThreadRoot "src\idle.c"),
    (Join-Path $RtThreadRoot "src\ipc.c"),
    (Join-Path $RtThreadRoot "src\irq.c"),
    (Join-Path $RtThreadRoot "src\kservice.c"),
    (Join-Path $RtThreadRoot "src\object.c"),
    (Join-Path $RtThreadRoot "src\scheduler.c"),
    (Join-Path $RtThreadRoot "src\thread.c"),
    (Join-Path $RtThreadRoot "src\timer.c"),
    (Join-Path $RtThreadRoot "components\finsh\shell.c"),
    (Join-Path $RtThreadRoot "components\finsh\msh.c"),
    (Join-Path $RtThreadRoot "components\finsh\msh_parse.c"),
    (Join-Path $RtThreadRoot "components\finsh\cmd.c")
)

$objects = @()
$index = 0
foreach ($source in $sources)
{
    if (-not (Test-Path -LiteralPath $source))
    {
        throw "源文件不存在：$source"
    }

    $index++
    $stem = [IO.Path]::GetFileNameWithoutExtension($source)
    $object = Join-Path $objectDir ("{0:D2}_{1}.o" -f $index, $stem)
    $args = @($commonFlags + $includeFlags + @("-std=gnu99", "-c", $source, "-o", $object))
    if ([IO.Path]::GetExtension($source) -eq ".S")
    {
        $args = @($commonFlags + $includeFlags + @("-x", "assembler-with-cpp", "-c", $source, "-o", $object))
    }

    & $gcc @args
    if ($LASTEXITCODE -ne 0)
    {
        throw "编译失败：$source"
    }
    $objects += $object
}

$elf = Join-Path $buildDir "rtthread_gorv32.elf"
$map = Join-Path $buildDir "rtthread_gorv32.map"
$linker = Join-Path $firmwareRoot "linker\gorv32_xip.ld"
$linkArgs = @(
    "-march=rv32imc", "-mabi=ilp32", "-mcmodel=medany", "-nostartfiles",
    "-Wl,--gc-sections", "-Wl,-Map=$map", "-T", $linker
) + $objects + @("-Wl,--start-group", "-lc", "-lgcc", "-Wl,--end-group", "-o", $elf)

& $gcc @linkArgs
if ($LASTEXITCODE -ne 0)
{
    throw "链接失败"
}

$bin = Join-Path $buildDir "rtthread_gorv32.bin"
$hex = Join-Path $buildDir "rtthread_gorv32.hex"
$lst = Join-Path $buildDir "rtthread_gorv32.lst"
& $objcopy -O binary $elf $bin
if ($LASTEXITCODE -ne 0) { throw "生成 BIN 失败" }
& $objcopy -O ihex $elf $hex
if ($LASTEXITCODE -ne 0) { throw "生成 HEX 失败" }
& $objdump -d -S $elf | Set-Content -LiteralPath $lst -Encoding ascii
if ($LASTEXITCODE -ne 0) { throw "生成反汇编清单失败" }

& $size -A $elf
if ($LASTEXITCODE -ne 0) { throw "读取固件尺寸失败" }

$elfHeader = (& $readelf -h $elf | Out-String)
if ($LASTEXITCODE -ne 0) { throw "读取 ELF 头失败" }
foreach ($pattern in @(
    'Class:\s+ELF32',
    'Machine:\s+RISC-V',
    'Entry point address:\s+0x80000000',
    'Flags:.*RVC.*soft-float ABI'))
{
    if ($elfHeader -notmatch $pattern)
    {
        throw "ELF ABI/入口自检失败：$pattern"
    }
}

$programHeaders = (& $readelf -l $elf | Out-String)
if ($LASTEXITCODE -ne 0) { throw "读取 ELF Program Header 失败" }
if ($programHeaders -notmatch '(?m)^\s*LOAD\s+\S+\s+0x80000000\s+0x80000000')
{
    throw "ELF XIP 段自检失败：未从逻辑地址 0x80000000 执行"
}
if ($programHeaders -notmatch '(?m)^\s*LOAD\s+\S+\s+0x40000000\s+0x8000[0-9a-f]+')
{
    throw "ELF RAM 初始化段自检失败：VMA/LMA 不符合 XIP 启动布局"
}

$symbols = (& $nm -n $elf | Out-String)
if ($LASTEXITCODE -ne 0) { throw "读取 ELF 符号失败" }
if ($symbols -notmatch '(?m)^80000000\s+\w\s+crtStart\s*$')
{
    throw "ELF 入口符号自检失败：crtStart != 0x80000000"
}
if ($symbols -notmatch '(?m)^40010000\s+\w\s+_sp\s*$')
{
    throw "ELF 栈顶自检失败：_sp != 0x40010000"
}
Write-Host "ELF 自检通过：RV32IMC/ILP32、XIP 0x80000000、RAM 0x40000000"

Get-FileHash -Algorithm SHA256 -LiteralPath $elf, $bin, $hex, $map, $lst |
    Select-Object Path, Hash
