#!/usr/bin/env python3
"""仅使用临时仓库及本地 bare 远程验证管理脚本。"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def run(*args, ok=True):
    result = subprocess.run(args, text=True, capture_output=True)
    assert (result.returncode == 0) == ok, (args, result.stdout, result.stderr)
    return result.stdout.strip()


def init(path, remote):
    path.mkdir(parents=True, exist_ok=True)
    run('git', 'init', '--bare', str(remote))
    run('git', 'init', '-b', 'main', str(path))
    run('git', '-C', str(path), 'config', 'user.name', 'Script Test')
    run('git', '-C', str(path), 'config', 'user.email', 'test@example.invalid')
    run('git', '-C', str(path), 'remote', 'add', 'origin', str(remote))


def main():
    for script in (ROOT / 'Scripts').iterdir():
        if script.suffix in ('.sh', '.bash'):
            run('bash', '-n', str(script))
    run('bash', str(ROOT / 'Scripts/sync_gowingame.sh'), ok=False)
    run('bash', str(ROOT / 'Scripts/setup_ros_dependencies.bash'), ok=False)
    with tempfile.TemporaryDirectory(prefix='fpga-workspace-scripts-') as directory:
        temp = Path(directory)
        os.environ['GIT_CONFIG_GLOBAL'] = os.devnull
        os.environ['GIT_CONFIG_NOSYSTEM'] = '1'
        root = temp / 'project'
        init(root, temp / 'main.git')
        shutil.copytree(ROOT / 'Scripts', root / 'Scripts')
        shutil.copytree(ROOT / 'manifests', root / 'manifests')
        shutil.copyfile(ROOT / '.gitignore', root / '.gitignore')
        (root / 'README.md').write_text('root\n')
        scopes = {'FPGA_ws': 'fpga_rtl', 'ugv_ws': 'ugv_ws', 'llm_ws': 'llm_ws'}
        push = root / 'Scripts/push_git.sh'
        for scope, name in scopes.items():
            repo = root / scope
            init(repo, temp / f'{name}.git')
            (repo / 'README.md').write_text(f'{scope}\n')
            run('bash', str(push), scope, 'initial workspace')
            assert run('git', '-C', str(repo), 'status', '--porcelain') == ''
        run('bash', str(push), 'main', 'initial main')
        tracked = run('git', '-C', str(root), 'ls-files').splitlines()
        assert not any(p.startswith(tuple(scope + '/' for scope in scopes)) for p in tracked)
        root_head = run('git', '-C', str(root), 'rev-parse', 'HEAD')
        for scope in scopes:
            run('git', '-C', str(root), 'add', '-f', scope)
            before = run('git', '-C', str(root), 'diff', '--cached')
            run('bash', str(push), 'main', 'must reject nested scope', ok=False)
            assert before == run('git', '-C', str(root), 'diff', '--cached')
            run('git', '-C', str(root), 'reset', 'HEAD', '--', scope)
        for scope in ('ros_ws', 'gowingame', 'unknown'):
            run('bash', str(push), scope, 'must reject', ok=False)
        run('bash', str(push), 'main', ' ', ok=False)
        for scope, name in scopes.items():
            peer = temp / f'peer-{scope}'
            run('git', 'clone', '-b', 'main', str(temp / f'{name}.git'), str(peer))
            run('git', '-C', str(peer), 'config', 'user.name', 'Test')
            run('git', '-C', str(peer), 'config', 'user.email', 'test@example.invalid')
            (peer / 'new.txt').write_text('remote update\n')
            run('git', '-C', str(peer), 'add', 'new.txt')
            run('git', '-C', str(peer), 'commit', '-m', 'remote update')
            run('git', '-C', str(peer), 'push')
        sync = root / 'Scripts/sync_workspaces.sh'
        run('bash', str(sync))
        for scope in scopes:
            assert (root / scope / 'new.txt').exists()
            shutil.rmtree(root / scope)
        config = temp / 'gitconfig'
        for name in scopes.values():
            run('git', 'config', '--file', str(config), f'url.{temp / (name + ".git")}.insteadOf',
                f'https://github.com/FPGAGowin2026/{name}.git')
            run('git', '--git-dir', str(temp / f'{name}.git'), 'symbolic-ref', 'HEAD', 'refs/heads/main')
        os.environ['GIT_CONFIG_GLOBAL'] = str(config)
        run('bash', str(sync))
        for scope in scopes:
            assert (root / scope / 'new.txt').exists()
        for script in ('sync_fpga_workspace.sh', 'sync_ros_packages.sh', 'sync_llm_workspace.sh'):
            run('bash', str(root / 'Scripts' / script))
        run('bash', str(sync), 'FPGA_ws', 'unknown', ok=False)
        assert run('git', '-C', str(root), 'rev-parse', 'HEAD') == root_head
        shutil.rmtree(root / 'ugv_ws/.git')
        run('bash', str(sync), 'ugv_ws', ok=False)
        run('bash', str(push), 'ugv_ws', 'must reject parent fallback', ok=False)
        assert (root / 'ugv_ws/new.txt').exists()
    print('PASS: syntax, manifest scopes, push isolation, clone, fast-forward and refusal guards')


if __name__ == '__main__':
    main()
