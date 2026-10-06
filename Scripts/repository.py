#!/usr/bin/env python3
"""从 repo manifest 读取工作区路径和远端。"""
from pathlib import Path, PurePosixPath
import sys
import xml.etree.ElementTree as ET


def repositories():
    root = ET.parse(Path(__file__).resolve().parent.parent / 'manifests/default.xml').getroot()
    remotes = {remote.attrib['name']: remote.attrib['fetch'] for remote in root.findall('remote')}
    default_remote = root.find('default').attrib['remote']
    result = {'main': ('.', '')}
    for project in root.findall('project'):
        path = PurePosixPath(project.attrib['path'])
        if path.is_absolute() or '..' in path.parts or len(path.parts) != 1 or str(path) in result:
            raise ValueError(f'invalid workspace path: {path}')
        fetch = remotes[project.get('remote', default_remote)].rstrip('/')
        result[str(path)] = (str(path), f"{fetch}/{project.attrib['name']}.git")
    return result


if __name__ == '__main__':
    repos = repositories()
    args = sys.argv[1:]
    if args == ['--list']:
        print('\n'.join(scope for scope in repos if scope != 'main'))
    elif len(args) == 1 and args[0] in repos:
        print(repos[args[0]][0])
    elif len(args) == 2 and args[0] == '--url' and args[1] in repos:
        print(repos[args[1]][1])
    else:
        sys.exit('用法：repository.py {--list|工作区|--url 工作区}；范围：main、FPGA_ws、ugv_ws、llm_ws')
