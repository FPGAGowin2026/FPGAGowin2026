"""校验当前轻量工程和引脚合同。"""
import argparse
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import sys


board = Path(__file__).resolve().parents[1] / 'boards/BX72_138K_robot'


def parse_project(project_file):
    """Parse Gowin's project XML, accepting its version=\"1\" declaration quirk."""
    source = project_file.read_text(encoding='utf-8-sig')
    source = re.sub(
        r'^(<\?xml\s+version\s*=\s*)(["\'])1\2',
        lambda match: match.group(1) + match.group(2) + '1.0' + match.group(2),
        source,
        count=1,
    )
    return ET.fromstring(source)


def active_project_files(project_file, project_root):
    entries = list(parse_project(project_file).iter('File'))
    if not entries:
        raise ValueError('Gowin project has no File entries')

    paths = []
    seen = set()
    for entry in entries:
        path = entry.get('path')
        if not path:
            raise ValueError('Gowin project File entry is missing its path')
        normalized = path.replace('\\', '/')
        if 'tmp' in normalized.casefold():
            raise ValueError(f'temporary path referenced by active project: {path}')
        if normalized.rsplit('/', 1)[-1].casefold() in {
                'ov5647_byte_to_pixel_ip.v', 'ov5647_csi2_rx_ip.v'}:
            raise ValueError(f'custom MIPI macro wrapper must not be active: {path}')
        if normalized.casefold() == 'src/pll_init.v':
            raise ValueError('duplicate PLL module must not be active: src/pll_init.v')
        if entry.get('enable') != '1':
            raise ValueError(f'project file is not enabled: {path}')
        key = normalized.casefold()
        if key in seen:
            raise ValueError(f'duplicate active project file: {path}')
        seen.add(key)
        if not (project_root / path).is_file():
            raise ValueError(f'active project file is missing: {path}')
        paths.append(normalized)

    if 'src/common/clock/pll_init.v' not in paths:
        raise ValueError('active project is missing src/common/clock/pll_init.v')
    return paths


def io_loc_assignments(cst_text):
    rows = re.findall(r'^\s*IO_LOC\s+"([^\"]+)"\s+([\w,]+)\s*;', cst_text, re.M)
    assignments = {}
    for signal, value in rows:
        if signal in assignments:
            raise ValueError(f'duplicate IO_LOC assignment: {signal}')
        assignments[signal] = value.split(',')
    return assignments


def unique_io_pins(cst_text):
    assignments = io_loc_assignments(cst_text)
    pins = [pin for values in assignments.values() for pin in values]
    if not pins:
        raise ValueError('CST contains no IO_LOC pin assignments')
    seen = set()
    duplicates = []
    for pin in pins:
        key = pin.casefold()
        if key in seen and pin not in duplicates:
            duplicates.append(pin)
        seen.add(key)
    if duplicates:
        raise ValueError(f'duplicate physical pin assignment(s): {", ".join(duplicates)}')
    return pins


def top_level_ports(top_text):
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', top_text, flags=re.S)
    header = re.search(r'\bmodule\s+robot_top\s*\((.*?)\)\s*;', source, re.S)
    if not header:
        raise ValueError('could not find robot_top ANSI port list')

    ports = []
    direction = None
    width = None
    for part in header.group(1).split(','):
        declaration = part.strip()
        if not declaration:
            continue
        direction_match = re.match(r'^(input|output|inout)\b', declaration)
        if direction_match:
            direction = direction_match.group(1)
            declaration = declaration[direction_match.end():].strip()
            width = None
        elif direction is None:
            raise ValueError(f'port declaration has no direction: {part!r}')

        while True:
            qualifier = re.match(r'^(?:wire|reg|logic|signed|unsigned)\b\s*', declaration)
            if not qualifier:
                break
            declaration = declaration[qualifier.end():]
        width_match = re.match(r'^\[(\d+)\s*:\s*(\d+)\]\s*', declaration)
        if width_match:
            width = (int(width_match.group(1)), int(width_match.group(2)))
            declaration = declaration[width_match.end():]
        name_match = re.match(r'^([A-Za-z_]\w*)\b', declaration)
        if not name_match:
            raise ValueError(f'could not parse {direction} port declaration: {part!r}')
        name = name_match.group(1)
        if width is None:
            ports.append(name)
        else:
            high, low = width
            step = -1 if high >= low else 1
            ports.extend(f'{name}[{index}]' for index in range(high, low + step, step))

    if not ports:
        raise ValueError('robot_top has no parsed ports')
    if len(ports) != len(set(ports)):
        raise ValueError('robot_top port list contains duplicate names')
    return ports


def validate_top_io_coverage(top_text, cst_text):
    ports = set(top_level_ports(top_text))
    locations = io_loc_assignments(cst_text)
    io_port_rows = re.findall(r'^\s*IO_PORT\s+"([^\"]+)"\s+([^;]+);', cst_text, re.M)
    io_port_options = {}
    for signal, options in io_port_rows:
        if signal in io_port_options:
            raise ValueError(f'duplicate IO_PORT assignment: {signal}')
        io_port_options[signal] = options
    io_types = {}
    for signal, options in io_port_options.items():
        match = re.search(r'\bIO_TYPE\s*=\s*([\w]+)', options)
        if match:
            io_types[signal] = match.group(1)

    stale_locations = sorted(set(locations) - ports)
    stale_options = sorted(set(io_port_options) - ports)
    if stale_locations or stale_options:
        raise ValueError(f'CST references non-top-level signal(s): '
                         f'IO_LOC={stale_locations}, IO_PORT={stale_options}')

    paired_differential_ports = {}
    for signal in ports:
        bit_index = re.search(r'(\[\d+\])$', signal)
        base = signal[:bit_index.start()] if bit_index else signal
        suffix = bit_index.group(1) if bit_index else ''
        if not base.endswith('_n'):
            continue
        positive = base[:-2] + suffix
        if (positive in ports and positive in locations and len(locations[positive]) == 2
                and io_types.get(positive, '').upper().endswith('D')):
            paired_differential_ports[signal] = positive

    missing_locations = sorted(signal for signal in ports
                               if signal not in locations and signal not in paired_differential_ports)
    if missing_locations:
        raise ValueError(f'top-level port(s) missing IO_LOC: {missing_locations}')
    missing_options = sorted(signal for signal in ports
                             if signal not in io_port_options and signal not in paired_differential_ports)
    if missing_options:
        raise ValueError(f'top-level port(s) missing IO_PORT: {missing_options}')

    missing_types = sorted(signal for signal in io_port_options if signal in ports and signal not in io_types)
    if missing_types:
        raise ValueError(f'top-level port(s) missing IO_TYPE: {missing_types}')

    for negative, positive in paired_differential_ports.items():
        if positive not in io_port_options:
            raise ValueError(f'differential pair {positive}/{negative} is missing IO_PORT')
    return len(ports)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=('all', 'udp'), default='udp')
    parser.add_argument('--candidate', type=Path, help='本地构建候选 JSON；不指定时只检查源码工作区')
    args = parser.parse_args()

    sys.path.insert(0, str(board / 'tools'))
    from verify_lite_project import verify
    result = verify(args.candidate)
    print(f"PASS: lite {result['active_files']} active files, {result['unique_pins']} unique pins, "
          f"{result['constrained_ports']} constrained top ports")


if __name__ == '__main__':
    main()
