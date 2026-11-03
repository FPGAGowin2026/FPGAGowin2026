"""按未修改SDC频率报告综合估计；最终时序以布局布线报告签核。"""
import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

# 这两个节点只串接PHY参考输入、HCLK和PLL，不直接驱动本板数据寄存器。
# 不把其缺少内部Fmax行记为通过；其引脚/专用时钟链仍待最终STA与板级测量。
REFERENCE_ONLY_TARGETS = {'rgmii_rx_clk_i', 'rx_sampling_pll/u_rx_hclk/CLKOUT'}


def check_required_instances(resources, required):
    """确认必需的板级功能仍有实现，避免基础链路被常量传播整体裁掉。"""
    root = ET.fromstring(resources)
    children = {node.get('name'): node for node in root.findall('SubModule')}
    result = []
    for name in required:
        node = children.get(name)
        count = sum(int(re.match(r'\d+', node.get(key, '0'))[0])
                    for key in ('T_Register', 'T_Lut', 'T_Bsram')) if node is not None else 0
        result.append(dict(instance=name, implemented_resources=count, passed=count > 0))
    return result


def clock_periods(sdc):
    periods = {}
    generated = []
    for line in sdc.splitlines():
        line = line.strip()
        targets = re.findall(r'\[get_(?:ports|pins)\s+\{([^}]+)\}\]', line)
        if line.startswith('create_clock ') and targets:
            period = re.search(r'-period\s+([\d.]+)', line)
            if period:
                periods[targets[-1]] = float(period[1])
        elif line.startswith('create_generated_clock ') and len(targets) >= 2:
            divide = re.search(r'-divide_by\s+(\d+)', line)
            if divide:
                generated.append((targets[0], targets[-1], int(divide[1])))
    for source, target, divide in generated:
        if source not in periods:
            raise ValueError(f'无法从原SDC解析generated clock源: {source}')
        periods[target] = periods[source] * divide
    return periods


def check(report, sdc):
    periods = clock_periods(sdc)
    section = report.split('<a name="performance">', 1)[1].split('</table>', 1)[0]
    clean = lambda value: html.unescape(re.sub(r'<[^>]*>', '', value)).strip()
    rows = [[clean(c) for c in re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>', row, re.S)]
            for row in re.findall(r'<tr[^>]*>(.*?)</tr>', section, re.S)]
    clocks = []
    reported_targets = set()
    for row in rows[1:]:
        name = row[1]
        target = name.removesuffix('.default_gen_clk')
        reported_targets.add(target)
        if target not in periods:
            raise ValueError(f'综合时钟没有原SDC频率对应: {name}')
        actual = float(row[3].replace('(MHz)', ''))
        required = 1000.0 / periods[target]
        if not math.isfinite(actual) or actual <= 0:
            raise ValueError(f'综合Fmax非法: {name}')
        clocks.append(dict(clock=name, required_MHz=required, actual_Fmax_MHz=actual,
                           passed=actual >= required))
    if not clocks:
        raise ValueError('综合报告缺少Max Frequency Summary')
    missing = sorted(set(periods) - REFERENCE_ONLY_TARGETS - reported_targets)
    slacks = [float(value) for value in re.findall(
        r'<td[^>]*>Slack</td>\s*<td[^>]*>([-+\d.]+)</td>', report)]
    if not slacks:
        raise ValueError('综合报告缺少详细Slack，不能放行PNR')
    return dict(kind='synthesis_estimate_only', passed=not missing and all(c['passed'] for c in clocks) and min(slacks) >= 0,
                clocks=clocks, worst_reported_slack_ns=min(slacks),
                missing_internal_clocks=missing,
                unreported_reference_clocks=sorted(set(periods) & REFERENCE_ONLY_TARGETS - reported_targets),
                reference_clock_signoff=False,
                production_SDC_signoff=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--sdc', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resources', type=Path)
    parser.add_argument('--synthesis-log', type=Path)
    parser.add_argument('--required-instance', action='append', default=[])
    parser.add_argument('--allow-estimated-timing-failure', action='store_true',
                        help='仅允许预估频率不足继续PNR，缺失时钟或实现仍阻止')
    args = parser.parse_args()
    try:
        result = check(args.report.read_text(encoding='utf-8'), args.sdc.read_text(encoding='utf-8-sig'))
        result['implementation_passed'] = not result['missing_internal_clocks']
        if args.required_instance:
            if args.resources is None:
                raise ValueError('必需模块检查缺少综合资源XML')
            result['required_instances'] = check_required_instances(
                args.resources.read_text(encoding='utf-8'), args.required_instance)
            result['passed'] &= all(item['passed'] for item in result['required_instances'])
            result['implementation_passed'] &= all(item['passed'] for item in result['required_instances'])
        if args.synthesis_log:
            result['undriven_application_nets'] = [line for line in
                args.synthesis_log.read_text(encoding='utf-8', errors='replace').splitlines()
                if 'does not have a driver' in line and '/vendor/' not in line.replace('\\', '/')]
            result['passed'] &= not result['undriven_application_nets']
            result['implementation_passed'] &= not result['undriven_application_nets']
    except (OSError, ValueError, IndexError, ET.ParseError) as error:
        result = dict(kind='synthesis_estimate_only', passed=False, error=str(error),
                      production_SDC_signoff=False)
    for name in ('report', 'sdc', 'resources', 'synthesis_log'):
        path = getattr(args, name)
        result[name + '_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest() if path and path.is_file() else None
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    failures = [c['clock'] for c in result.get('clocks', []) if not c['passed']]
    print(f"SYNTHESIS_TIMING_GATE passed={int(result['passed'])} failed_clocks={','.join(failures)} "
          f"missing_internal={','.join(result.get('missing_internal_clocks', []))} "
          f"missing_instances={','.join(item['instance'] for item in result.get('required_instances', []) if not item['passed'])} "
          f"undriven_nets={len(result.get('undriven_application_nets', []))} "
          f"worst_slack_ns={result.get('worst_reported_slack_ns')} error={result.get('error', '')}")
    acceptable_estimate = args.allow_estimated_timing_failure and result.get('implementation_passed', False)
    raise SystemExit(0 if result['passed'] or acceptable_estimate else 1)


if __name__ == '__main__':
    main()
