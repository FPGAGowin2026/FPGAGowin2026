"""检查BX72速度控制、位姿融合的活动清单、物理引脚、时钟及哈希。"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

BOARD = Path(__file__).resolve().parents[1] / "boards/BX72_138K_robot"
ROOT = BOARD.parents[1]
sys.path.insert(0, str(ROOT / 'Scripts'))
from verify_project import validate_top_io_coverage, unique_io_pins
from check_synthesis_timing import clock_periods
from firmware_identity import firmware_input_sha256


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(candidate=None):
    project = BOARD / 'BX72_138K_robot.gprj'
    tree = ET.parse(project)
    files = [p.get('path') for p in tree.iter('File') if p.get('enable', '1') == '1']
    if len(files) != len(set(files)):
        raise ValueError('活动清单含重复文件')
    for name in files:
        path = (BOARD / name).resolve()
        if not path.is_relative_to(BOARD / 'src') or not path.is_file():
            raise ValueError('活动源缺失或越出src: ' + name)
    required = {
        'src/softcore/vendor/gorv32_fp/riscv_fp_core.v', 'src/softcore/bx72_nav_platform.sv',
        'src/softcore/bx72_nav_boot_pkg.sv', 'src/top/bx72_lite_robot_core.sv',
        'src/ethernet/dma/bx72_telemetry_mux.sv',
        'src/ethernet/dma/bx72_telemetry_packet_cdc.sv',
        'src/navigation/bx72_lite_guard.sv', 'src/motor/mecanum_motor_control.sv',
        'src/navigation/bx72_polar_reduce.sv',
        'src/video/camera/ov5647_csi2_capture.v', 'src/common/ddr3/udp_camera_ddr.v',
        'src/ethernet/dma/eth_udp_transport.sv', 'src/ethernet/eth_udp_gmii/eth_udp_gmii.v'
    }
    if not required <= set(files):
        raise ValueError('轻量活动实现未列入GPRJ: ' + str(sorted(required - set(files))))
    rom = (BOARD / 'src/softcore/bx72_nav_boot_pkg.sv').read_text(encoding='utf-8-sig')
    marker = re.search(r'(?m)^// firmware-source-sha256=([0-9a-f]{64})\s*$', rom)
    if not marker or marker.group(1) != firmware_input_sha256(BOARD / 'src/firmware/lite') or \
            not re.search(r'\bROM_ABI_VERSION\s*=\s*2\s*;', rom):
        raise ValueError('固件源码与ROM身份或接口不匹配；先重新生成ROM')
    forbidden = ('bx72_lite_cpu.sv', 'bx72_lite_platform.sv', 'bx72_gorv32_fp_', 'bx72_local_navigation.sv',
                 'bx72_bspline_planner.sv', 'EKF/robot_fusion.sv',
                 '/ip/gowin_isp/', 'src/top/bx72_robot_core.sv')
    if any(part in name for name in files for part in forbidden):
        raise ValueError('旧宽数据通路仍在活动清单')
    top = (BOARD / 'src/top/robot_top.v').read_text(encoding='utf-8-sig')
    cst = (BOARD / 'src/constraint/BX72_138K_robot.cst').read_text(encoding='utf-8-sig')
    if 'telemetry_u/' in cst:
        raise ValueError('CST仍引用已替换的单遥测实例')
    ports = validate_top_io_coverage(top, cst)
    pins = len(unique_io_pins(cst))
    if (ports, pins) != (125, 125):
        raise ValueError(f'物理端口合同变化: ports={ports} pins={pins}')
    if not re.search(r'IMAGE_WIDTH\s*=\s*640\s*,\s*IMAGE_HEIGHT\s*=\s*360', top):
        raise ValueError('首板视频规格应为640x360')
    sdc = (BOARD / 'src/constraint/BX72_138K_robot.sdc').read_text(encoding='utf-8-sig')
    periods = clock_periods(sdc)
    expected = {'clk50m': 20., 'rgmii_rx_clk_i': 8., 'camera_mipi_clk_p': 4.571429,
                'rx_sampling_pll/u_pll/PLL_inst/CLKOUT0': 8.,
                'Gowin_PLL/u_pll/PLL_inst/CLKOUT0': 8.,
                'Gowin_PLL/u_pll/PLL_inst/CLKOUT1': 25.,
                'Gowin_PLL/u_pll/PLL_inst/CLKOUT3': 30.,
                'ov5647_capture/camera_pixel_pll/u_pll/PLL_inst/CLKOUT0': 22.857,
                'ov5647_capture/camera_dphy_rx/DPHY_RX_INST_u_idesx8_Inst3_CLKDIV/CLKOUT': 18.285716}
    for target, period in expected.items():
        if target not in periods or abs(periods[target] - period) > 1e-6:
            raise ValueError(f'实际时钟合同缺失/变化: {target}')
    if 'isp_clk100m' in sdc:
        raise ValueError('SDC仍引用已退出的ISP100时钟')
    if 'bx72_lite_robot_core robot(' not in top or '.cam_clk(camera_pixel_clk)' not in top:
        raise ValueError('顶层未接入轻量核心/真实相机像素时钟')
    if '.clkout1(control_clk40m)' not in top or '.control_clk(control_clk40m)' not in top:
        raise ValueError('CPU/运动未接实际PLL CLKOUT1输出')
    if '.USE_CONTROL_BRIDGE(1)' not in top or '.command_bus_clk_i(loc_clk50m)' not in top:
        raise ValueError('接收命令未通过50MHz包总线送到40MHz控制域')
    if '.reply_ready(rx_phase_tx[20]' in top or '.reply_valid(udp_reply_valid &&' in top:
        raise ValueError('RX训练期间仍静默丢弃已校验ACK')
    platform = (BOARD / 'src/softcore/bx72_nav_platform.sv').read_text(encoding='utf-8-sig')
    linker = (BOARD / 'src/firmware/lite/link.ld').read_text(encoding='utf-8-sig')
    if "RAM_LIMIT  = 32'h40008000" not in platform or not re.search(r'RAM\s*\(rwx\).*LENGTH\s*=\s*32K', linker):
        raise ValueError('32KiB物理RAM与链接地址不一致')
    pll = (BOARD / 'src/ethernet/ip/gowin_pll/gowin_pll_mod.v').read_text(encoding='utf-8-sig')
    if not re.search(r'PLL_inst\.ODIV1_SEL\s*=\s*25\s*;', pll):
        raise ValueError('CPU PLL实际输出分频不是25')
    module_owner = {}
    for name in files:
        if Path(name).suffix.lower() not in ('.v', '.sv'):
            continue
        text = (BOARD / name).read_text(encoding='utf-8-sig')
        text = re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
        for module in re.findall(r'\bmodule\s+(\w+)\b', text):
            if module in module_owner:
                raise ValueError(f'重名模块{module}: {name}, {module_owner[module]}')
            module_owner[module] = name
    inputs = {name: digest(BOARD / name) for name in files}
    inputs[project.name] = digest(project)
    for path in sorted((BOARD / 'src/firmware/lite').glob('*')):
        if path.is_file():
            inputs[path.relative_to(BOARD).as_posix()] = digest(path)
    checked_candidate = None
    if candidate:
        record = json.loads(Path(candidate).read_text(encoding='utf-8-sig'))
        for key in ('source_inputs', 'evidence_sha256'):
            for name, expected_hash in record[key].items():
                if digest(BOARD / name) != expected_hash:
                    raise ValueError(f'{key}哈希不一致: {name}')
        if digest(BOARD / record['bitstream']) != record['bitstream_sha256']:
            raise ValueError('位流哈希不一致')
        checked_candidate = str(candidate)
    return dict(passed=True, architecture='nav_grid_v1', ram_capacity_bytes=32768,
                active_files=len(files), unique_pins=pins, constrained_ports=ports,
                module_count=len(module_owner), clock_periods_ns=periods,
                source_inputs=inputs, candidate_checked=checked_candidate,
                external_interface_signoff=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        result = verify(args.candidate)
    except (OSError, ValueError, AssertionError, KeyError) as error:
        result = dict(passed=False, error=str(error))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'source_inputs'}, ensure_ascii=False))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
