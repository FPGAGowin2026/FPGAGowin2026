"""核对详细时序、DDR 时钟和厂商模型边界；支持新构建或保留的交付报告。"""
import argparse
import html
import json
from pathlib import Path
import re
import math
import sys

board = Path(__file__).resolve().parents[1] / 'boards/BX72_138K_robot'
sys.path.insert(0, str(board.parents[1] / 'Scripts'))
from analyze_board_reports import table
parser = argparse.ArgumentParser()
parser.add_argument('--reports', type=Path, default=board / 'impl/pnr')
parser.add_argument('--output', type=Path, default=board / 'build/checks/timing.json')
parser.add_argument('--sdc', type=Path, default=board / 'src/constraint/BX72_138K_robot.sdc',
                    help='用于核对摄像头时钟约束的活动 SDC')
parser.add_argument('--build-log', type=Path)
parser.add_argument('--resource-profile', choices=('legacy', 'telemetry', 'single_comm', 'udp', 'radar_slam'), default='legacy',
                    help='udp 沿用原活动工程的器件80%上限；其他配置仅供历史报告审计')
parser.add_argument('--ddr-audit', type=Path, default=board / 'sim/checks/ddr_model_audit_baseline.json',
                    help='已审查的厂商 DDR 边界记录；可指定从 Git 提取的只读基线副本')
parser.add_argument('--require-rx-clock', action='store_true', help='要求新增 RX 域的 125 MHz 约束与 Fmax')
parser.add_argument('--variant', choices=('udp',), default='udp')
parser.add_argument('--require-board-signoff', action='store_true',
                    help='外部接口预算未验收时返回失败；内部时序通过不能替代上板验收')
args = parser.parse_args()
content = (args.reports / 'BX72_138K_robot_tr_content.html').read_text(encoding='utf-8-sig')


def read_camera_clock_rates(sdc_path):
    names = ('camera_byte_clk', 'camera_pixel_clk')
    periods = {}
    errors = []
    commands = [line.split('#', 1)[0].strip()
                for line in sdc_path.read_text(encoding='utf-8-sig').splitlines()]
    # Resolve the supported port-sourced divider from its explicit primary clock.
    # Unsupported/ambiguous sources remain missing constraints and fail closed.
    primary = {}
    for command in commands:
        if not re.match(r'^create_clock\b', command):
            continue
        port = re.search(r'\[get_ports\s+\{([^}]+)\}\]', command)
        period = re.search(r'-period\s+([\d.]+)\b', command)
        if port and period:
            primary.setdefault(port.group(1), []).append(float(period.group(1)))
    for command in commands:
        if not re.match(r'^create_(?:generated_)?clock\b', command):
            continue
        name_match = re.search(r'-name\s+(?:\{([^}]+)\}|(\S+))', command)
        period_match = re.search(r'-period\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+))\b', command)
        if not name_match:
            continue
        name = name_match.group(1) or name_match.group(2)
        if name not in names:
            continue
        if command.startswith('create_generated_clock'):
            source = re.search(r'-source\s+\[get_ports\s+\{([^}]+)\}\]', command)
            divider = re.search(r'-divide_by\s+(\d+)\b', command)
            source_periods = primary.get(source.group(1), []) if source else []
            if (len(source_periods) != 1 or not divider or int(divider.group(1)) <= 0
                    or re.search(r'-(?:multiply_by|edges|edge_shift|invert|master_clock)\b', command)):
                errors.append(f'unsupported generated SDC clock constraint: {name}')
                continue
            period = source_periods[0] * int(divider.group(1))
        elif period_match:
            period = float(period_match.group(1))
        else:
            continue
        if name in periods:
            errors.append(f'duplicate SDC clock constraint: {name}')
            continue
        if not math.isfinite(period) or period <= 0:
            errors.append(f'invalid SDC period for {name}: {period}')
            continue
        periods[name] = 1000.0 / period
    for name in names:
        if name not in periods:
            errors.append(f'missing SDC clock constraint: {name}')
    return periods, errors


def report_frequency(row, column):
    value = row.get(column, '')
    match = re.fullmatch(
        r'\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*(?:\([^)]*\))?\s*',
        value,
    )
    if not match:
        raise ValueError(f'invalid {column}: {value!r}')
    result = float(match.group(1))
    if not math.isfinite(result):
        raise ValueError(f'non-finite {column}: {value!r}')
    return result


minimum = {}
errors = []
for kind in ('setup', 'hold', 'recovery', 'removal'):
    rows = table(content, kind.title() + '_Slack_Table')
    values = [float(row['Path Slack']) for row in rows]
    if any(not math.isfinite(value) for value in values):
        errors.append(f'{kind}: non-finite slack')
    minimum[kind] = min(values, default=None)
    if minimum[kind] is None or minimum[kind] < 0:
        errors.append(f'{kind}: {minimum[kind]}')
clocks = {row['Clock Name']: row for row in table(content, 'Max_Frequency_Report')}
camera_clock_rates = {}
if args.variant == 'udp':
    camera_clock_rates, camera_sdc_errors = read_camera_clock_rates(args.sdc)
    errors.extend(camera_sdc_errors)
expected_clocks = {'clk50m_in': 50, 'ddr_clk50m': 50,
                   'ddr_clk200m': 200, 'ddr_ui_clk': 50, 'eth_clk125m': 125,
                   'sensor_clk33m': 33.333}
expected_clocks.update(camera_clock_rates)
if args.variant == 'udp' and (args.require_rx_clock or args.reports.resolve() == (board / 'impl/pnr').resolve()):
    expected_clocks['eth_rx_clk'] = 125
for clock, rate in expected_clocks.items():
    if clock not in clocks:
        errors.append(f'missing/wrong clock: {clock}')
        continue
    tolerance = 0.0005 if clock in camera_clock_rates else 0.0
    try:
        constrained_rate = report_frequency(clocks[clock], 'Constraint')
        actual_fmax = report_frequency(clocks[clock], 'Actual Fmax')
    except ValueError as error:
        errors.append(f'{clock}: {error}')
        continue
    if abs(constrained_rate - rate) > tolerance + 1e-9:
        errors.append(f'wrong clock constraint: {clock} (report {constrained_rate:g} MHz, '
                      f'expected {rate:.6f} MHz)')
    if actual_fmax + tolerance + 1e-9 < rate:
        errors.append(f'Fmax below constraint: {clock}')

# 所有报告时钟都必须满足自己的目标，不能只验证命名白名单。
for clock, row in clocks.items():
    try:
        target = report_frequency(row, 'Constraint')
        actual = report_frequency(row, 'Actual Fmax')
        if target <= 0 or actual < target:
            errors.append(f'all-clock Fmax failure: {clock}: {actual} < {target}')
    except ValueError as error:
        errors.append(f'{clock}: {error}')

# 汇总检查覆盖全部负端点，不能只依赖默认前 25 条详细路径。
tns = table(content, 'Total_Negative_Slack_Report')
if not tns:
    errors.append('missing total negative slack summary')
for row in tns:
    if (not math.isfinite(float(row['Endpoints TNS'])) or
            float(row['Endpoints TNS']) != 0 or int(row['Number of Endpoints']) != 0):
        errors.append(f'negative endpoints: {row}')

# 最大延迟例外可能单独列出，也要检查详细报告中的全部 Slack 项。
reported_slacks = []
for value in re.findall(r'<td[^>]*>\s*Slack:?\s*</td>\s*<td[^>]*>(.*?)</td>', content, re.S):
    plain = html.unescape(re.sub('<[^>]*>', '', value)).strip()
    match = re.match(r'(-?\d+(?:\.\d+)?)', plain)
    if match:
        reported_slacks.append(float(match[1]))
if reported_slacks and min(reported_slacks) < 0:
    errors.append(f'negative detailed/exception slack: {min(reported_slacks)}')

known_warnings = []
review_warnings = []
resources = {}
# 遥测的待发快照、串行移位、雷达收齐缓存和电机邮箱需要额外寄存器。
# 显式选择新功能预算，默认门限不变；存储块、DSP 和锁存器均不扩额。
resource_limits = {'Logic': 44105, 'Register': 31305, 'BSRAM': 156, 'DSP': 92, 'latches': 1}
if args.resource_profile in ('telemetry', 'single_comm'):
    resource_limits['Logic'] += 10000
    # 当前 EKF 静止约束和输入诊断比原遥测版本多 1117 个寄存器；
    # 将寄存器增量额度调至 11000 后，上限仍仅占 GW5AT-138C 的约 30.4%。
    resource_limits['Register'] += 11000
if args.resource_profile == 'single_comm':
    # 合并基线含雷达定位/打滑融合，DSP=67*2+28+5/2=164.5（Gowin 物理块口径）。
    # 新版上限不超过该功能基线；旧 legacy/telemetry 的 92 门槛保持不变。
    resource_limits['DSP'] = 164.5
if args.resource_profile in ('udp', 'radar_slam'):
    # UDP 沿用任务开始前活动工程的80%门限，数值不变；
    # radar_slam 旧名称仅保留用于历史报告审计。
    resource_limits = {'Logic': 110592, 'Register': 111276, 'BSRAM': 272,
                       'DSP': 238.4, 'latches': 1}
if args.build_log:
    log = args.build_log.read_text(encoding='utf-8-sig')
    # 告警完整列入报告，资源/时序门禁按实际数值判定；工具ERROR仍直接失败。
    for line in log.splitlines():
        if re.search(r'\bERROR\b', line):
            errors.append(line)
        if 'WARN' in line:
            if ('(TA1132)' in line and "'flash_clk'" in line) or (
                    '(PR1014)' in line and 'FLASH_QSPI_CLK_in' in line):
                known_warnings.append(line)
            elif ('(CV0016)' in line and 'src\\top\\robot_top.v' in line
                  and 'Input debug_uart_rx_i is unused' in line):
                known_warnings.append(line)
            else:
                review_warnings.append(line)
    resource_path = args.reports / 'BX72_138K_robot.rpt.txt'
    resource_text = resource_path.read_text(encoding='utf-8-sig')
    # 对全部有器件容量的资源检查used/available，不局限于五项预算。
    for label, used, capacity in re.findall(
            r'^\s*([^|\r\n]+?)\s*\|\s*(\d+(?:\.\d+)?)/(\d+(?:\.\d+)?)',
            resource_text, re.M):
        if float(used) > float(capacity):
            errors.append(f'device resource capacity exceeded: {label.strip()}={used}/{capacity}')
    for name in ('Logic', 'Register', 'BSRAM', 'DSP'):
        limit = resource_limits[name]
        # Gowin 的 DSP 汇总可使用半个 DSP 计数（例如 53.5/298）。
        match = re.search(r'^\s*' + name + r'\s*\|\s*(\d+(?:\.\d+)?)/', resource_text, re.M)
        if not match:
            errors.append(f'missing resource: {name}')
        else:
            value = float(match[1])
            resources[name] = int(value) if value.is_integer() else value
            if value > limit:
                errors.append(f'resource growth: {name}={resources[name]} > {limit}')
    match = re.search(r'--Logic Register as Latch\s*\|\s*(\d+)/', resource_text)
    if not match or int(match[1]) > resource_limits['latches']:
        errors.append('missing latch count or latch count above baseline 1')
    else:
        resources['latches'] = int(match[1])

budgets = json.loads((board / 'src/constraint/interface_timing.json').read_text(encoding='utf-8'))
pending = [name for name, item in budgets['interfaces'].items() if not item['verified']]
if args.require_board_signoff and pending:
    errors.append(f'board interface signoff pending: {pending}')

# 保留原负路径的边界审查证据，不将模型分组视为加密 IP 内部 CDC 仿真。
audit = json.loads(args.ddr_audit.read_text(encoding='utf-8'))['paths']
domains = {'clk50m_in': 'reference', 'ddr_clk50m': 'reference',
           'ddr_clk200m': 'memory', 'ddr_ui_clk': 'user'}
prefix = 'ddr3_ctrl_2port/DDR3_Memory_Interface_Top/'
for row in audit:
    source = domains.get(row['From Clock'].split(':')[0])
    target = domains.get(row['To Clock'].split(':')[0])
    if not (source and target and source != target and row['From Node'].startswith(prefix)
            and row['To Node'].startswith(prefix)):
        errors.append(f'unsupported vendor boundary: {row["kind"]} {row["Path Number"]}')
result = dict(passed=not errors, worst_slack_ns=minimum,
    ddr_clocks={name: clocks.get(name) for name in ('ddr_clk50m', 'ddr_clk200m', 'ddr_ui_clk')},
    camera_sdc_clocks=camera_clock_rates,
    audited_baseline_negative_paths=len(audit), errors=errors,
    clocks=clocks, total_negative_slack=tns, resources=resources,
    resource_profile=args.resource_profile, resource_limits=resource_limits,
    known_warnings=known_warnings, review_warnings=review_warnings, board_signoff_pending=pending,
    detailed_slack_count=len(reported_slacks),
    encrypted_ip_cdc_verified_by_custom_simulation=False)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps({key: result[key] for key in
    ('passed', 'worst_slack_ns', 'resources', 'resource_profile', 'resource_limits',
     'known_warnings', 'review_warnings', 'board_signoff_pending', 'errors')}, ensure_ascii=False))
raise SystemExit(0 if result['passed'] else 1)
