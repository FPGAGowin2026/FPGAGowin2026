"""构建 GoRV32 位姿融合、本地建图与导航固件，并生成片上 ROM 初始化。"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from firmware_identity import firmware_input_sha256


BOARD = Path(__file__).resolve().parents[1] / "boards/BX72_138K_robot"
FIRMWARE = BOARD / "src/firmware/lite"
ROM_MAX_BYTES = 64 * 1024
RAM_BYTES = 32 * 1024
STACK_BYTES = 4 * 1024
CPU_CLOCK_HZ = 40000000
CONTROL_CLOCK_HZ = 40000000


def wsl_path(path):
    """将Windows绝对路径转换为WSL挂载路径。"""
    path = Path(path).resolve()
    if not path.drive:
        raise ValueError(f"固件构建路径必须位于Windows盘符下: {path}")
    return "/mnt/" + path.drive[0].lower() + path.as_posix()[2:]


def build(output_dir, package_path):
    source_sha256 = firmware_input_sha256(FIRMWARE)
    output_dir.mkdir(parents=True, exist_ok=True)
    elf = output_dir / "bx72_navigation.elf"
    binary = output_dir / "bx72_navigation.bin"
    commands = []

    def run(arguments, capture_path=None):
        commands.append(arguments)
        result = subprocess.run(
            ["wsl.exe", "--", *arguments], capture_output=True, text=True
        )
        (output_dir / "build_commands.json").write_text(
            json.dumps(commands, indent=2), encoding="utf-8"
        )
        output = result.stdout + result.stderr
        (output_dir / f"command_{len(commands):02d}.log").write_text(
            output, encoding="utf-8"
        )
        if capture_path is not None:
            capture_path.write_text(result.stdout, encoding="utf-8")
        if result.returncode:
            raise RuntimeError(output)
        return result.stdout

    version = run(["riscv64-unknown-elf-gcc", "--version"])
    compiler_version = re.search(r"(\d+)\.\d+", version)
    if not compiler_version:
        raise RuntimeError("无法解析riscv64-unknown-elf-gcc版本")
    # GCC10按旧ISA命名把CSR归在I扩展内，尚不接受独立_zicsr后缀。
    march = "rv32imf" if int(compiler_version[1]) < 11 else "rv32imf_zicsr"
    cflags = [
        "-march=" + march, "-mabi=ilp32f", "-Os", "-std=gnu11",
        "-ffp-contract=off", "-fsingle-precision-constant",
        "-ffreestanding", "-fno-builtin", "-fno-pic", "-fno-pie",
        "-fno-stack-protector", "-fno-asynchronous-unwind-tables",
        "-ffunction-sections", "-fdata-sections", "-msmall-data-limit=0",
        "-nostdlib", "-nostartfiles", "-Wall", "-Wextra", "-Werror",
        "-fstack-usage",
    ]
    sources = [FIRMWARE / "start.S", *sorted(FIRMWARE.glob("*.c"))]
    required_sources = {"start.S", "main.c", "manual_control.c", "fusion.c", "csm.c",
                        "trig.c", "runtime.c", "navigation.c", "local_grid.c", "local_path.c"}
    missing = required_sources - {path.name for path in sources}
    if missing:
        raise RuntimeError("缺少固件源码: " + ", ".join(sorted(missing)))
    objects = []
    for source in sources:
        obj = output_dir / f"{source.stem}.o"
        run([
            "riscv64-unknown-elf-gcc", *cflags, "-c", wsl_path(source),
            "-o", wsl_path(obj),
        ])
        objects.append(wsl_path(obj))

    run([
        "riscv64-unknown-elf-gcc", *cflags,
        "-Wl,--gc-sections,--no-relax",
        "-Wl,-Map=" + wsl_path(output_dir / "bx72_navigation.map"),
        "-T", wsl_path(FIRMWARE / "link.ld"), *objects, "-lgcc",
        "-o", wsl_path(elf),
    ])
    run([
        "riscv64-unknown-elf-objcopy", "-O", "binary",
        wsl_path(elf), wsl_path(binary),
    ])
    run([
        "riscv64-unknown-elf-objdump", "-d", "-h", wsl_path(elf),
    ], output_dir / "bx72_navigation.dis")
    undefined = run(["riscv64-unknown-elf-nm", "-u", wsl_path(elf)])
    if undefined.strip():
        raise RuntimeError("存在未解析符号: " + undefined)

    symbol_text = run(
        ["riscv64-unknown-elf-nm", "-n", wsl_path(elf)],
        output_dir / "symbols.txt",
    )
    symbols = {
        name: int(address, 16)
        for address, name in re.findall(r"^([0-9a-fA-F]+)\s+\w\s+(\S+)$", symbol_text, re.M)
    }
    required = ("__data_start", "__bss_end", "__stack_bottom", "__stack_top")
    if not all(name in symbols for name in required):
        raise RuntimeError("链接脚本未导出RAM预算符号")
    static_bytes = symbols["__bss_end"] - symbols["__data_start"]
    stack_reserved = symbols["__stack_top"] - symbols["__stack_bottom"]
    if static_bytes > RAM_BYTES - STACK_BYTES or stack_reserved != STACK_BYTES:
        raise RuntimeError("固件静态数据或保留栈超过32KiB RAM预算")

    stack_frames = []
    for usage_file in sorted(output_dir.glob("*.su")):
        for line in usage_file.read_text(encoding="utf-8").splitlines():
            fields = line.split("\t")
            if len(fields) == 3:
                stack_frames.append({
                    "function": fields[0], "bytes": int(fields[1]), "kind": fields[2]
                })

    image = binary.read_bytes()
    if not image or len(image) > ROM_MAX_BYTES:
        raise RuntimeError(f"ROM镜像长度非法: {len(image)}")
    if firmware_input_sha256(FIRMWARE) != source_sha256:
        raise RuntimeError("固件构建期间源码变化，拒绝生成ROM")
    # 8KiB ROM bank按实际链接镜像配置，避免把41KiB镜像扩大到64KiB。
    rom_capacity = max(16384, (len(image) + 8191) // 8192 * 8192)
    padded = image.ljust(rom_capacity, b"\0")
    lines = [
        "// 自动生成：Scripts/build_lite_firmware.py；禁止手改。",
        "// binary SHA256=" + hashlib.sha256(image).hexdigest(),
        "// firmware-source-sha256=" + source_sha256,
        "package bx72_nav_boot_pkg;",
        "    localparam integer ROM_ABI_VERSION = 2;",
        f"    localparam integer ROM_BYTES = {rom_capacity};",
        "    localparam integer ROM_WORDS = ROM_BYTES / 4;",
        "    function automatic [31:0] boot_word(input [13:0] address);",
        "        begin",
        "            case (address)",
    ]
    for offset in range(0, rom_capacity, 4):
        value = int.from_bytes(padded[offset:offset + 4], "little")
        if value:
            lines.append(
                f"                14'd{offset // 4}: boot_word = 32'h{value:08x};"
            )
    lines += [
        "                default: boot_word = 32'd0;",
        "            endcase",
        "        end",
        "    endfunction",
        "endpackage",
        "",
    ]
    package_path.parent.mkdir(parents=True, exist_ok=True)
    package_path.write_text("\n".join(lines), encoding="utf-8")

    manifest = {
        "architecture": "local_grid_path_v1",
        "firmware_source_sha256": source_sha256,
        "compiler": version.splitlines()[0],
        "commands": commands,
        "isa": march,
        "abi": "ilp32f",
        "cpu_clock_hz": CPU_CLOCK_HZ,
        "control_clock_hz": CONTROL_CLOCK_HZ,
        "rom_bytes": len(image),
        "rom_capacity_bytes": rom_capacity,
        "ram_capacity_bytes": RAM_BYTES,
        "ram_static_bytes": static_bytes,
        "stack_reserved_bytes": stack_reserved,
        "ram_free_below_stack_bytes": RAM_BYTES - static_bytes - stack_reserved,
        "stack_frames": stack_frames,
        "stack_note": "编译器栈帧不含libgcc调用链；保留容量不等于实测栈高水位。",
        "binary_sha256": hashlib.sha256(image).hexdigest(),
        "package_sha256": hashlib.sha256(package_path.read_bytes()).hexdigest(),
        "sources": {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(FIRMWARE.iterdir()) if path.is_file()
        },
    }
    (output_dir / "firmware_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    print(json.dumps({
        key: manifest[key]
        for key in ("rom_bytes", "ram_static_bytes", "binary_sha256", "package_sha256")
    }))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--package", type=Path,
        default=BOARD / "src/softcore/bx72_nav_boot_pkg.sv",
    )
    arguments = parser.parse_args()
    build(arguments.output_dir.resolve(), arguments.package.resolve())
