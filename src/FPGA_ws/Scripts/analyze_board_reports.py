"""提取Gowin资源和详细时序证据，不依据TNS汇总判定时序通过。"""
import argparse
import hashlib
import html
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET


def table(text, anchor):
    # 多条 report_timing 会生成带编号的子表；父锚点后第一张表不代表全局最差值。
    anchors = re.findall(r'name="(' + re.escape(anchor) + r'\d+)"', text) or [anchor]
    result = []
    for item in anchors:
        marker = 'name="' + item + '"'
        if marker not in text:
            continue
        tail = text.split(marker, 1)[1]
        # 空的Recovery/Removal段会写Nothing to report，不能越过下一锚点
        # 把Minimum Pulse Width表误当成该段的路径表。
        section = re.split(r'<a\s+name="', tail, maxsplit=1)[0]
        match = re.search(r"<table[^>]*>(.*?)</table>", section, re.S)
        if not match:
            continue
        body = match.group(1)
        rows = []
        for row in re.findall(r"<tr[^>]*>(.*?)</tr>", body, re.S):
            cells = re.findall(r"<t[dh][^>]*>(.*?)</t[dh]>", row, re.S)
            rows.append([html.unescape(re.sub(r"<[^>]*>", "", c)).strip() for c in cells])
        result.extend(dict(zip(rows[0], r)) for r in rows[1:])
    return result


def analyze(board, impl):
    name = board.name
    timing = (impl / 'pnr' / (name + '_tr_content.html')).read_text(encoding='utf-8')
    report = (impl / 'pnr' / (name + '.rpt.txt')).read_text(encoding='utf-8')
    resources = {}
    for label in ('Logic', 'Register', 'BSRAM', 'DSP'):
        resources[label] = re.search(r'^  ' + label + r'\s+\| ([\d.]+)', report, re.M).group(1)
    match = re.search(r'\((\d+) LUT, (\d+) ALU', report)
    resources.update(LUT=match[1], ALU=match[2])
    resources['latches'] = re.search(r'Logic Register as Latch\s+\| (\d+)', report)[1]
    result = dict(board=name, resources=resources,
                  fmax=table(timing, 'Max_Frequency_Report'),
                  tns=table(timing, 'Total_Negative_Slack_Report'))
    for kind in ('Setup', 'Hold', 'Recovery', 'Removal'):
        rows = table(timing, kind + '_Slack_Table')
        result[kind.lower()] = rows
        result['worst_' + kind.lower()] = min(float(r['Path Slack']) for r in rows) if rows else None
    modules = ET.parse(impl / 'gwsynthesis' / (name + '_syn_rsc.xml')).getroot()
    result['module_resources'] = [e.attrib for e in modules.iter()
                                  if e.get('name') in ('control', 'camera_half_view', 'radar_point_map')
                                  or 'linebuffer' in e.get('name', '') or 'bitmap' in e.get('name', '')]
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('board', type=Path)
    p.add_argument('--impl', type=Path)
    p.add_argument('--input-root', type=Path, help='归档构建对应的冻结输入目录')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = analyze(args.board, args.impl or args.board / 'impl')
    input_root = args.input_root or args.board
    result['current_inputs'] = {str(f.relative_to(input_root)): hashlib.sha256(f.read_bytes()).hexdigest()
                                for f in sorted((input_root / 'src').rglob('*')) if f.is_file()}
    result['current_inputs'][args.board.name + '.gprj'] = hashlib.sha256(
        (input_root / (args.board.name + '.gprj')).read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k in
                      ('board', 'resources', 'worst_setup', 'worst_hold', 'worst_recovery', 'worst_removal')}, ensure_ascii=False))
