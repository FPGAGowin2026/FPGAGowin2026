"""编译并运行 BX72 控制、局部路径、融合、CSM 及三角主机回归。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / 'boards/BX72_138K_robot'
FIRMWARE = BOARD / 'src/firmware/lite'
TESTS = BOARD / 'sim/lite'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'gcc'), help='主机C编译器')
    parser.add_argument('--output-dir', type=Path,
                        default=BOARD / 'build/velocity_only/host')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / 'summary.json'
    summary_path.unlink(missing_ok=True)

    def run(command, name):
        result = subprocess.run([str(value) for value in command], cwd=BOARD,
                                capture_output=True, text=True, encoding='utf-8',
                                errors='replace', timeout=120)
        (output / (name + '.log')).write_text(result.stdout + result.stderr, encoding='utf-8')
        if result.returncode:
            raise RuntimeError(f'{name} 失败（{result.returncode}），见 {output / (name + ".log")}')
        return result.stdout.strip()

    common = ['-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-I' + str(FIRMWARE)]
    fusion = [FIRMWARE / 'fusion.c', FIRMWARE / 'trig.c']
    no_asm = ['-DBX72_HOST_TEST_NO_RV32_ASM']
    specs = [
        ('manual_control', [], ['manual_control_test.c'], [FIRMWARE / 'manual_control.c']),
        ('local_path', [], ['local_path_test.c'], [FIRMWARE / 'local_path.c']),
        ('local_grid_unknown', [], ['local_grid_unknown_test.c'],
         [FIRMWARE / 'local_grid.c', FIRMWARE / 'trig.c']),
        ('local_navigation_rebase', ['-DBX72_NAV_HOST_TEST'], ['local_navigation_rebase_test.c'],
         [FIRMWARE / 'local_grid.c', FIRMWARE / 'local_path.c',
          FIRMWARE / 'manual_control.c', FIRMWARE / 'trig.c']),
        ('navigation_fragment', ['-DBX72_NAV_HOST_TEST'], ['navigation_fragment_test.c'],
         [FIRMWARE / 'local_grid.c', FIRMWARE / 'local_path.c',
          FIRMWARE / 'manual_control.c', FIRMWARE / 'trig.c']),
        ('fusion', no_asm, ['a_fusion_host_test.c'], fusion),
        ('trig', [], ['a_trig_error_host_test.c'], [FIRMWARE / 'trig.c']),
        ('csm_reference', no_asm, ['csm_reference_test.c'], fusion),
        ('fusion_csm_contract', no_asm, ['fusion_csm_contract_test.c'], fusion),
        ('polar_pose', no_asm, ['polar_pose_test.c'], [*fusion, FIRMWARE / 'csm.c']),
        ('lidar_bearing', no_asm, ['lidar_bearing_test.c'], [*fusion, FIRMWARE / 'csm.c']),
    ]
    all_inputs = sorted({*FIRMWARE.glob('*.c'), *FIRMWARE.glob('*.h'),
                         Path(__file__).resolve(),
                         *(TESTS / test for _, _, tests, _ in specs for test in tests)})
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in all_inputs}
    compiler = run([args.cc, '--version'], 'compiler_version').splitlines()[0]
    records = []
    for name, flags, tests, sources in specs:
        executable = output / (name + ('.exe' if os.name == 'nt' else ''))
        command = [args.cc, *common, *flags, *[TESTS / test for test in tests], *sources,
                   '-lm', '-o', executable]
        run(command, name + '_compile')
        result = run([executable], name + '_run')
        print(f'PASS {name}: {result}', flush=True)
        test_inputs = [*[TESTS / test for test in tests], *sources]
        records.append({'name': name, 'passed': True, 'compile_command': list(map(str, command)),
                        'output': result, 'inputs_sha256': {
                            str(path.relative_to(BOARD)): hashlib.sha256(path.read_bytes()).hexdigest()
                            for path in test_inputs}})
    after = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in all_inputs}
    if hashes != after:
        raise RuntimeError('测试期间源码发生变化，请冻结输入后重跑')
    summary_path.write_text(json.dumps({'passed': True, 'compiler': compiler,
                                       'input_sha256': hashes, 'tests': records},
                                       ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'PASS {len(records)} 项主机回归：{summary_path}', flush=True)


if __name__ == '__main__':
    main()
