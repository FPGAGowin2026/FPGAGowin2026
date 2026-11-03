#!/usr/bin/env python3
"""BX72 独立构建：直接使用活动源码、记录输入哈希及检查内部时序。"""
import argparse
from contextlib import contextmanager, nullcontext
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from firmware_identity import firmware_input_sha256

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / 'boards/BX72_138K_robot'
NAME = BOARD.name
PNR_TIMEOUT_SECONDS = 1800.0
ROUND_TIMEOUT_SECONDS = 1800.0
GOWIN_MUTEX_NAME = r'Local\BX72_Gowin_Global'
OPTIONS = {
    'FPGA_PLACE_OPTION': '1', 'FPGA_ROUTE_OPTION': '2', 'FPGA_ROUTE_MAXFAN': '23',
    'FPGA_CLOCK_ROUTE_ORDER': '0', 'FPGA_TIMING_DRIVEN': '1', 'FPGA_CORRECT_HOLD': '1',
    'FPGA_REPLICATE_RESOURCES': '0', 'FPGA_OPT_GOAL': 'area', 'FPGA_MAP_OPTION': '2',
    'FPGA_RETIMING_RESOURCE': 'none', 'FPGA_SYN_MAX_FANOUT': '1000',
    'FPGA_SYN_NETLIST_HIERARCHY': '1',
}


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


class WindowsProcessJob:
    """在运行前绑定 Job；由系统记录后代身份，不依赖可复用的 PID。"""

    def __init__(self):
        import ctypes
        from ctypes import wintypes

        class BasicLimit(ctypes.Structure):
            _fields_ = [('per_process_time', ctypes.c_longlong), ('per_job_time', ctypes.c_longlong),
                        ('flags', wintypes.DWORD), ('min_working_set', ctypes.c_size_t),
                        ('max_working_set', ctypes.c_size_t), ('active_limit', wintypes.DWORD),
                        ('affinity', ctypes.c_size_t), ('priority', wintypes.DWORD),
                        ('scheduling', wintypes.DWORD)]

        class IoCounters(ctypes.Structure):
            _fields_ = [(name, ctypes.c_ulonglong) for name in
                        ('read_ops', 'write_ops', 'other_ops', 'read_bytes', 'write_bytes', 'other_bytes')]

        class ExtendedLimit(ctypes.Structure):
            _fields_ = [('basic', BasicLimit), ('io', IoCounters),
                        ('process_memory', ctypes.c_size_t), ('job_memory', ctypes.c_size_t),
                        ('peak_process_memory', ctypes.c_size_t), ('peak_job_memory', ctypes.c_size_t)]

        class Accounting(ctypes.Structure):
            _fields_ = [(name, ctypes.c_longlong) for name in
                        ('user', 'kernel', 'period_user', 'period_kernel')] + [
                        (name, wintypes.DWORD) for name in
                        ('page_faults', 'total', 'active', 'terminated')]

        class ThreadEntry(ctypes.Structure):
            _fields_ = [('size', wintypes.DWORD), ('usage', wintypes.DWORD),
                        ('tid', wintypes.DWORD), ('owner', wintypes.DWORD),
                        ('base_priority', wintypes.LONG), ('delta_priority', wintypes.LONG),
                        ('flags', wintypes.DWORD)]

        self.ctypes, self.Accounting, self.ThreadEntry = ctypes, Accounting, ThreadEntry
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        self.kernel.CreateJobObjectW.argtypes = [wintypes.LPVOID, wintypes.LPCWSTR]
        self.kernel.CreateJobObjectW.restype = wintypes.HANDLE
        self.kernel.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int,
                                                        wintypes.LPVOID, wintypes.DWORD]
        self.kernel.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
        self.kernel.QueryInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int,
                                                          wintypes.LPVOID, wintypes.DWORD,
                                                          ctypes.POINTER(wintypes.DWORD)]
        self.kernel.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
        self.kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        self.kernel.CreateToolhelp32Snapshot.argtypes = [wintypes.DWORD, wintypes.DWORD]
        self.kernel.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
        self.kernel.Thread32First.argtypes = [wintypes.HANDLE, ctypes.POINTER(ThreadEntry)]
        self.kernel.Thread32Next.argtypes = [wintypes.HANDLE, ctypes.POINTER(ThreadEntry)]
        self.kernel.OpenThread.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        self.kernel.OpenThread.restype = wintypes.HANDLE
        self.kernel.GetProcessIdOfThread.argtypes = [wintypes.HANDLE]
        self.kernel.GetProcessIdOfThread.restype = wintypes.DWORD
        self.kernel.ResumeThread.argtypes = [wintypes.HANDLE]
        self.kernel.ResumeThread.restype = wintypes.DWORD
        self.handle = self.kernel.CreateJobObjectW(None, None)
        self.assigned = False
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        limits = ExtendedLimit()
        limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not self.kernel.SetInformationJobObject(self.handle, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
            error = ctypes.WinError(ctypes.get_last_error())
            self.kernel.CloseHandle(self.handle)
            raise error

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.kernel.CloseHandle(self.handle)

    def start(self, process):
        if not self.kernel.AssignProcessToJobObject(self.handle, int(process._handle)):
            raise self.ctypes.WinError(self.ctypes.get_last_error())
        self.assigned = True
        snapshot = self.kernel.CreateToolhelp32Snapshot(0x00000004, 0)  # TH32CS_SNAPTHREAD
        if snapshot == self.ctypes.c_void_p(-1).value:
            raise self.ctypes.WinError(self.ctypes.get_last_error())
        try:
            entry = self.ThreadEntry()
            entry.size = self.ctypes.sizeof(entry)
            found = self.kernel.Thread32First(snapshot, self.ctypes.byref(entry))
            tids = []
            while found:
                if entry.owner == process.pid:
                    tids.append(entry.tid)
                found = self.kernel.Thread32Next(snapshot, self.ctypes.byref(entry))
            error = self.ctypes.get_last_error()
            if error != 18:  # ERROR_NO_MORE_FILES
                raise self.ctypes.WinError(error)
        finally:
            self.kernel.CloseHandle(snapshot)
        if len(tids) != 1 or process.poll() is not None:
            raise RuntimeError(f'挂起进程的主线程不唯一或已退出: pid={process.pid}, tids={tids}')
        thread = self.kernel.OpenThread(0x0002 | 0x0800, False, tids[0])
        if not thread:
            raise self.ctypes.WinError(self.ctypes.get_last_error())
        try:
            if self.kernel.GetProcessIdOfThread(thread) != process.pid or process.poll() is not None:
                raise RuntimeError(f'挂起线程的进程身份已改变: pid={process.pid}')
            previous = self.kernel.ResumeThread(thread)
            if previous == 0xffffffff:
                raise self.ctypes.WinError(self.ctypes.get_last_error())
            if previous != 1:
                raise RuntimeError(f'挂起线程的计数异常: pid={process.pid}, count={previous}')
        finally:
            self.kernel.CloseHandle(thread)

    def active(self):
        info = self.Accounting()
        if not self.kernel.QueryInformationJobObject(self.handle, 1, self.ctypes.byref(info),
                                                     self.ctypes.sizeof(info), None):
            raise self.ctypes.WinError(self.ctypes.get_last_error())
        return info.active


def stop_windows_process_tree(job, root_pid):
    """只停止本轮 Job 中的进程，并确认成员已经退出。"""
    try:
        if job.active() and not job.kernel.TerminateJobObject(job.handle, 1):
            raise job.ctypes.WinError(job.ctypes.get_last_error())
        deadline = time.monotonic() + 6
        while job.active():
            if time.monotonic() >= deadline:
                return False, f'TREE_STOP_FAILED root={root_pid} active={job.active()}'
            time.sleep(.05)
        return True, f'TREE_STOPPED root={root_pid} active=0'
    except OSError as exc:
        return False, f'TREE_STOP_FAILED root={root_pid}: {exc}'


@contextmanager
def gowin_global_mutex():
    """本机所有Gowin入口共用命名Mutex；排队不消耗工具执行预算。"""
    if os.name != 'nt':
        yield 0.0
        return
    import ctypes
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateMutexW.argtypes = [wintypes.LPVOID, wintypes.BOOL, wintypes.LPCWSTR]
    kernel.CreateMutexW.restype = wintypes.HANDLE
    kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel.WaitForSingleObject.restype = wintypes.DWORD
    kernel.ReleaseMutex.argtypes = [wintypes.HANDLE]
    kernel.ReleaseMutex.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    handle = kernel.CreateMutexW(None, False, GOWIN_MUTEX_NAME)
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    acquired = False
    queued = time.monotonic()
    try:
        print(f'GOWIN_MUTEX_WAIT {GOWIN_MUTEX_NAME}', flush=True)
        status = kernel.WaitForSingleObject(handle, 0xffffffff)
        # WAIT_ABANDONED同样取得所有权，后续仍需按完整构建执行。
        if status not in (0, 0x80):
            raise ctypes.WinError(ctypes.get_last_error())
        acquired = True
        # 前owner被强制终止时Mutex会变abandoned，但其工具进程可能仍活着。
        # 即使不是abandoned也核对一次；不把取得Mutex本身当作旧树已退出证据。
        wait_existing_gowin()
        waited = time.monotonic() - queued
        print(f'GOWIN_MUTEX_ACQUIRED queue_seconds={waited:.3f}', flush=True)
        yield waited
    finally:
        if acquired:
            kernel.ReleaseMutex(handle)
        kernel.CloseHandle(handle)


def wait_existing_gowin():
    """持有本机会话Mutex时等待旧Gowin退出，不结束身份未知的进程。"""
    script = r"""
$sessionId = [Diagnostics.Process]::GetCurrentProcess().SessionId
@(Get-CimInstance Win32_Process | Where-Object {
    $_.Name -eq 'gw_sh.exe' -and $_.SessionId -eq $sessionId
} | ForEach-Object {
    [pscustomobject]@{ pid=[int]$_.ProcessId; age=([DateTime]::Now-$_.CreationDate).TotalSeconds }
}) | ConvertTo-Json -Compress
"""
    while True:
        probe = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', script],
                               capture_output=True, text=True, timeout=15, check=True)
        rows = json.loads(probe.stdout) if probe.stdout.strip() else []
        if isinstance(rows, dict):
            rows = [rows]
        if not rows:
            return
        for row in rows:
            print(f'GOWIN_MUTEX_OLD_TREE pid={row["pid"]} age_seconds={row["age"]:.3f}', flush=True)
        time.sleep(.5)


def run_gowin(command, log, cwd, env, stage, timeout=PNR_TIMEOUT_SECONDS,
              round_timeout=ROUND_TIMEOUT_SECONDS):
    # 先校验参数，再排队；预算起点在取得Mutex之后、Popen之前。
    if not math.isfinite(float(round_timeout)) or not 0 < round_timeout <= ROUND_TIMEOUT_SECONDS:
        raise ValueError('综合+PNR共用超时必须为有限正数且不超过1800秒')
    if stage == 'pnr' and (timeout is None or not math.isfinite(float(timeout)) or timeout <= 0):
        raise ValueError('PNR 必须配置有限的正数秒级超时')
    with gowin_global_mutex() as waited:
        env = dict(env, FPGA_PYTHON_FOR_GATES=sys.executable)
        result = _run_gowin_locked(command, log, cwd, env, stage, timeout, round_timeout)
        result.gowin_mutex_name = GOWIN_MUTEX_NAME if os.name == 'nt' else None
        result.gowin_mutex_queue_seconds = waited
        return result


def _run_gowin_locked(command, log, cwd, env, stage, timeout=PNR_TIMEOUT_SECONDS,
                      round_timeout=ROUND_TIMEOUT_SECONDS):
    """综合启动前建立同一截止时间，综合和PNR共用，不因阶段切换重置。"""
    pnr_started = None
    pnr_started_at_unix_ms = None
    pnr_ended_at_unix_ms = None
    pnr_seconds = None
    timed_out = False
    round_timed_out = False
    tree_termination_failed = False
    tree_termination_diagnostic = None
    tree_stopped = False
    pnr_process_id = None
    text = ''
    with (WindowsProcessJob() if os.name == 'nt' else nullcontext(None)) as job, \
         log.open('w', encoding='utf-8') as stream, log.open('r', encoding='utf-8', errors='replace') as reader:
        round_started_at_unix_ms = int(time.time() * 1000)
        round_started = time.monotonic()
        round_deadline = round_started + round_timeout
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                   creationflags=0x00000004 if job else 0)  # CREATE_SUSPENDED
        pnr_process_id = process.pid
        try:
            if job:
                job.start(process)
            while True:
                text += reader.read()
                if pnr_started is None:
                    marker = re.search(r'BUILD_START pnr (\d+)', text)
                    if marker:
                        pnr_started_at_unix_ms = int(marker[1])
                        pnr_started = time.monotonic() - max(
                            0, time.time() - pnr_started_at_unix_ms / 1000)
                    elif stage == 'pnr' and 'Reading netlist file:' in text:
                        pnr_started = time.monotonic()
                        pnr_started_at_unix_ms = int(time.time() * 1000)
                finished = re.search(r'BUILD_TIME pnr ([\d.]+) seconds', text)
                if finished:
                    pnr_seconds = float(finished[1])
                elapsed = time.monotonic() - pnr_started if pnr_started is not None else None
                # BUILD_TIME 只是工具报告，不能作为看门狗依据：工具可能先写出该行后继续挂住。
                round_timed_out = time.monotonic() >= round_deadline
                if round_timed_out or (timeout is not None and elapsed is not None and elapsed >= timeout):
                    timed_out = True
                    pnr_seconds = elapsed
                    if os.name == 'nt':
                        stopped, tree_termination_diagnostic = stop_windows_process_tree(job, process.pid)
                        tree_stopped = stopped
                        tree_termination_failed = not stopped
                        if not stopped and process.poll() is None:
                            # 尽力回收直接子进程；构建结果仍标记树停止失败，调用者必须先处理诊断。
                            process.kill()
                    elif process.poll() is None:
                        process.kill()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    text += reader.read()
                    pnr_ended_at_unix_ms = int(time.time() * 1000)
                    label = 'ROUND_TIMEOUT' if round_timed_out else 'PNR_TIMEOUT'
                    limit = round_timeout if round_timed_out else timeout
                    if tree_termination_failed:
                        stream.write(f'\n{label}: 超过 {limit} 秒；进程树停止失败，PID={process.pid}。\n'
                                     f'{tree_termination_diagnostic}\n请先确认该进程树已退出，再启动其他变体。\n')
                    else:
                        stream.write(f'\n{label}: 超过 {limit} 秒，进程树已停止。\n'
                                     f'{tree_termination_diagnostic or ""}\n')
                    stream.flush()
                    break
                if process.poll() is not None:
                    # 子进程退出到最后一次读取之间可能还有末尾输出。
                    text += reader.read()
                    round_timed_out = time.monotonic() >= round_deadline
                    timed_out |= round_timed_out
                    marker = re.search(r'BUILD_START pnr (\d+)', text)
                    if pnr_started is None and marker:
                        pnr_started_at_unix_ms = int(marker[1])
                        pnr_started = time.monotonic() - max(
                            0, time.time() - pnr_started_at_unix_ms / 1000)
                    elif stage == 'pnr' and pnr_started is None and 'Reading netlist file:' in text:
                        pnr_started = time.monotonic()
                        pnr_started_at_unix_ms = int(time.time() * 1000)
                    if pnr_started is not None:
                        elapsed = time.monotonic() - pnr_started
                        pnr_seconds = elapsed
                        if timeout is not None and elapsed >= timeout:
                            timed_out = True
                    if pnr_started is not None:
                        pnr_ended_at_unix_ms = int(time.time() * 1000)
                    break
                wait_seconds = min(0.05, max(0.001, round_deadline - time.monotonic()))
                if timeout is not None and pnr_started is not None:
                    remaining = timeout - (time.monotonic() - pnr_started)
                    if remaining > 0:
                        wait_seconds = min(wait_seconds, remaining)
                try:
                    process.wait(timeout=wait_seconds)
                except subprocess.TimeoutExpired:
                    pass
        finally:
            if job and job.assigned and not tree_stopped:
                # 先确认 Job 已空，再等待主进程句柄，避免异常时提前释放 Mutex。
                stopped, diagnostic = stop_windows_process_tree(job, process.pid)
                tree_termination_diagnostic = diagnostic
                tree_termination_failed |= not stopped
                while not stopped:
                    time.sleep(.05)
                    stopped, diagnostic = stop_windows_process_tree(job, process.pid)
                    tree_termination_diagnostic = diagnostic
            if process.poll() is None:
                process.kill()
            process.wait(timeout=10)
    return argparse.Namespace(returncode=process.returncode, pnr_timed_out=timed_out,
                              round_timed_out=round_timed_out,
                              round_timeout_seconds=round_timeout,
                              round_elapsed_seconds=time.monotonic() - round_started,
                              round_started_at_unix_ms=round_started_at_unix_ms,
                              round_deadline_at_unix_ms=round_started_at_unix_ms + int(round_timeout * 1000),
                              # 保留实测精度，避免已超限值被毫秒舍入回门限本身。
                              pnr_elapsed_seconds=pnr_seconds,
                              pnr_started_at_unix_ms=pnr_started_at_unix_ms,
                              pnr_ended_at_unix_ms=pnr_ended_at_unix_ms,
                              pnr_tree_termination_failed=tree_termination_failed,
                              pnr_tree_termination_diagnostic=tree_termination_diagnostic,
                              pnr_process_id=pnr_process_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gowin-shell', default=os.environ.get('GOWIN_SHELL', 'gw_sh'))
    parser.add_argument('--stage', choices=('full', 'syn', 'pnr'), default='full')
    parser.add_argument('--variant', choices=('udp',), default='udp')
    parser.add_argument('--round-timeout', type=float, default=ROUND_TIMEOUT_SECONDS,
                        help='综合启动前起算，综合+PNR共用硬截止时间，上限1800秒')
    parser.add_argument('--pnr-timeout', type=float, default=PNR_TIMEOUT_SECONDS,
                        help='PNR 阶段墙钟硬超时秒数（默认1800；综合+PNR另共用1800秒总截止时间）')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--incremental', choices=('none', 'auto', 'place', 'pnr'), default='none')
    parser.add_argument('--force', action='store_true', help='忽略精确缓存并重跑；仍保留实现检查点')
    args = parser.parse_args()
    if args.stage != 'full':
        parser.error('本轮只允许full，禁止拆分或重启规避综合+PNR总截止时间')
    if not math.isfinite(args.round_timeout) or not 0 < args.round_timeout <= ROUND_TIMEOUT_SECONDS:
        parser.error('--round-timeout 必须为有限正数且不超过1800秒')
    if not math.isfinite(args.pnr_timeout) or args.pnr_timeout <= 0:
        parser.error('--pnr-timeout 必须是有限的正数秒')
    shell = shutil.which(args.gowin_shell)
    if shell is None:
        parser.error('找不到高云命令行；通过 --gowin-shell 或 GOWIN_SHELL 指定 gw_sh.exe')
    out = (args.output_dir or BOARD / 'build' / f'bx72_{args.variant}').resolve()
    if out == BOARD or out == ROOT or not out.is_relative_to(BOARD / 'build'):
        parser.error('--output-dir 必须位于 BX72 的 build/ 内，避免覆盖源码或其他板工程')
    out.mkdir(parents=True, exist_ok=True)
    # 全板互斥，独立输出目录也不能同时启动 Gowin。
    lock = BOARD / 'build' / 'gowin.lock'
    try:
        handle = lock.open('x', encoding='utf-8')
    except FileExistsError:
        parser.error(f'{lock} 已存在；确认对应进程退出后再移除此锁')
    with handle:
        handle.write(str(os.getpid()))
    try:
        build(args, Path(shell).resolve(), out)
    finally:
        lock.unlink()


def build(args, shell, out):
    pnr_timeout = getattr(args, 'pnr_timeout', PNR_TIMEOUT_SECONDS)
    project_name = f'{NAME}.gprj'
    tree = ET.parse(BOARD / project_name)
    rom = BOARD / 'src/softcore/bx72_nav_boot_pkg.sv'
    if not rom.is_file():
        raise SystemExit('缺少当前固件ROM；先运行 Scripts/build_lite_firmware.py')
    rom_text = rom.read_text(encoding='utf-8-sig')
    marker = re.search(r'(?m)^// firmware-source-sha256=([0-9a-f]{64})\s*$', rom_text)
    if not marker or marker.group(1) != firmware_input_sha256(BOARD / 'src/firmware/lite') or \
            not re.search(r'\bROM_ABI_VERSION\s*=\s*2\s*;', rom_text):
        raise SystemExit('固件源码与ROM身份或接口不匹配；重新运行 Scripts/build_lite_firmware.py')
    inputs = set()
    inputs.update((BOARD / item.get('path')).resolve() for item in tree.iter('File'))
    # 厂商 IP 同目录的初始化和配置也进入身份校验。
    for path in list(inputs):
        if '/ip/' in path.as_posix():
            inputs.update(p for p in path.parent.rglob('*') if p.is_file())
    inputs.update((BOARD / project_name, ROOT / 'Scripts/build.tcl', Path(__file__).resolve(),
                   ROOT / 'Scripts/firmware_identity.py', ROOT / 'Scripts/check_synthesis_timing.py',
                   ROOT / 'Scripts/check_ddr_timing.py', ROOT / 'Scripts/analyze_board_reports.py',
                   BOARD / 'sim/checks/ddr_model_audit_baseline.json',
                   BOARD / 'src/constraint/interface_timing.json'))
    if any(p.name == 'bx72_nav_platform.sv' for p in inputs):
        # C源与启动/链接合同一同绑定ROM产物，避免只保存生成包而丢失固件身份。
        inputs.update(p for p in (BOARD / 'src/firmware/lite').iterdir() if p.is_file())
    hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(inputs)}
    options = {key: os.environ.get(key, default) for key, default in OPTIONS.items()}
    identity = {'variant': args.variant, 'inputs': hashes, 'options': options, 'tool': str(shell), 'tool_sha256': digest(shell)}
    # 增量模式也纳入身份，避免显式对照实验命中另一配置的缓存。
    identity['incremental'] = args.incremental
    identity['cpu_velocity_control'] = 1
    identity['round_timeout_seconds'] = args.round_timeout
    fingerprint = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    result_path = out / 'build_result.json'
    old = json.loads(result_path.read_text(encoding='utf-8')) if result_path.exists() else {}
    if old.get('variant', args.variant) != args.variant:
        raise SystemExit('输出目录已有另一通信版本的结果；请为两版指定不同的 --output-dir')
    valid = old.get('fingerprint') == fingerprint and bool(old.get('artifacts')) and all(
        (out / p).is_file() and digest(out / p) == value for p, value in old.get('artifacts', {}).items())
    syn = out / f'impl/gwsynthesis/{NAME}.vg'
    pnr = out / 'impl/pnr'
    syn_ok = valid and old.get('tool_passed') and old.get('synthesis_passed') and syn.is_file()
    old_pnr_elapsed = old.get('pnr_elapsed_seconds')
    if old_pnr_elapsed is None:
        old_pnr_elapsed = old.get('stage_seconds', {}).get('pnr', float('inf'))
    pnr_ok = (valid and old.get('timing_gate_passed') and not old.get('pnr_timed_out', True)
              and not old.get('round_timed_out', True)
              and old.get('round_elapsed_seconds', float('inf')) <= args.round_timeout
              and old.get('pnr_timeout_seconds') is not None
              and isinstance(old_pnr_elapsed, (int, float)) and math.isfinite(old_pnr_elapsed)
              and old_pnr_elapsed <= pnr_timeout
              and (pnr / f'{NAME}.fs').is_file())
    if not args.force and ((args.stage == 'syn' and syn_ok) or (args.stage != 'syn' and pnr_ok)):
        print(f'CACHE HIT: {out}; 输入、工具、选项和产物 SHA-256 一致，跳过 {args.stage}', flush=True)
        return
    if args.stage == 'pnr' and not syn_ok:
        raise SystemExit('PNR 拒绝复用：综合网表缺失或输入/选项/产物已变化；先运行 --stage full/syn')
    stage = args.stage
    for item in tree.iter('File'):
        item.set('path', (BOARD / item.get('path')).resolve().as_posix())
    project = out / project_name
    project_bytes = ET.tostring(tree.getroot(), encoding='utf-8', xml_declaration=True)
    if not project.is_file() or project.read_bytes() != project_bytes:
        project.write_bytes(project_bytes)
    log = out / 'build.log'
    result = {'fingerprint': fingerprint, **identity, 'stage': stage, 'synthesis_passed': bool(syn_ok),
              'synthesis_timing_gate_passed': False, 'pnr_skipped_synthesis_timing': False,
              'source_top_sha256': hashes[(BOARD / 'src/top/robot_top.v').relative_to(ROOT).as_posix()],
              'timing_gate_passed': False, 'artifacts': {}, 'hardware_tested': False,
              'pnr_timeout_seconds': pnr_timeout, 'pnr_timed_out': False}
    write_json(result_path, result)  # 失败/中断时不会留下旧的通过状态。
    env = dict(os.environ, **options, FPGA_BOARD=NAME, FPGA_BUILD_STAGE=stage,
               FPGA_PROJECT=str(project), FPGA_VARIANT=args.variant, FPGA_INCREMENTAL=args.incremental)
    # 清除阶段输出的旧报告；保留 .p/.pr 作为增量候选。
    expected = [syn] if stage == 'syn' else [pnr / f'{NAME}{suffix}' for suffix in
                                            ('.fs', '.rpt.html', '.tr.html', '_tr_content.html')]
    for path in expected:
        path.unlink(missing_ok=True)
    if stage != 'syn':
        for name in ('timing_gate.json', 'timing_gate.log', 'synthesis_timing_gate.json'):
            (out / name).unlink(missing_ok=True)
    started = time.monotonic()
    print(f'BUILD {NAME} {stage}: {out}; incremental={args.incremental}', flush=True)
    process = run_gowin([str(shell), str(ROOT / 'Scripts/build.tcl')], log, out, env, stage,
                        timeout=pnr_timeout, round_timeout=args.round_timeout)
    for key in ('round_timed_out', 'round_timeout_seconds', 'round_elapsed_seconds',
                'round_started_at_unix_ms', 'round_deadline_at_unix_ms',
                'gowin_mutex_name', 'gowin_mutex_queue_seconds'):
        result[key] = getattr(process, key)
    result['pnr_timed_out'] = process.pnr_timed_out
    result['tool_returncode'] = process.returncode
    result['pnr_elapsed_seconds'] = process.pnr_elapsed_seconds
    result['pnr_started_at_unix_ms'] = getattr(process, 'pnr_started_at_unix_ms', None)
    result['pnr_ended_at_unix_ms'] = getattr(process, 'pnr_ended_at_unix_ms', None)
    result['pnr_tree_termination_failed'] = getattr(process, 'pnr_tree_termination_failed', False)
    result['pnr_tree_termination_diagnostic'] = getattr(process, 'pnr_tree_termination_diagnostic', None)
    result['pnr_process_id'] = getattr(process, 'pnr_process_id', None)
    synthesis_gate = out / 'synthesis_timing_gate.json'
    if synthesis_gate.is_file():
        result['synthesis_timing_gate_passed'] = json.loads(synthesis_gate.read_text(encoding='utf-8'))['passed']
    result['pnr_skipped_synthesis_timing'] = 'SYNTHESIS_TIMING_REJECT' in log.read_text(encoding='utf-8', errors='replace')
    result['elapsed_seconds'] = round(time.monotonic() - started, 3)
    text = log.read_text(encoding='utf-8', errors='replace')
    result['reported_stage_seconds'] = {name: float(value) for name, value in
                                        re.findall(r'BUILD_TIME (syn|pnr) ([\d.]+) seconds', text)}
    result['stage_seconds'] = dict(result['reported_stage_seconds'])
    if stage == 'pnr' and process.pnr_elapsed_seconds is not None:
        result['stage_seconds']['pnr'] = process.pnr_elapsed_seconds
    result['incremental_fallback'] = 'PA2136' in text
    result['implicit_synthesis'] = stage == 'pnr' and 'GowinSynthesis start' in text
    result['synthesis_passed'] = syn.is_file() and (syn_ok or 'BUILD_TIME syn ' in text)
    result['tool_passed'] = (process.returncode == 0 and not process.pnr_timed_out
                             and not result['pnr_tree_termination_failed']
                             and not re.search(r'\b(?:ERROR|ERR)\s*\(', text))
    required = [syn] if stage == 'syn' else [pnr / f'{NAME}{suffix}' for suffix in ('.fs', '.rpt.html', '.tr.html')]
    result['tool_passed'] &= all(p.is_file() for p in required) and f'BUILD_TIME {"syn" if stage == "syn" else "pnr"} ' in text
    if stage != 'syn' and result['tool_passed']:
        gate_log = out / 'timing_gate.log'
        resource_profile = 'udp'
        with gate_log.open('w', encoding='utf-8') as stream:
            gate = subprocess.run([sys.executable, str(ROOT / 'Scripts/check_ddr_timing.py'),
                '--reports', str(pnr), '--resource-profile', resource_profile, '--variant', args.variant,
                *(['--require-rx-clock'] if args.variant == 'udp' else []),
                '--build-log', str(log), '--output', str(out / 'timing_gate.json')],
                stdout=stream, stderr=subprocess.STDOUT)
        result['timing_gate_passed'] = gate.returncode == 0
    changed_inputs = []
    for rel, expected_hash in hashes.items():
        try:
            if digest(ROOT / rel) != expected_hash:
                changed_inputs.append(rel)
        except OSError:
            changed_inputs.append(rel)
    if changed_inputs:
        result['changed_inputs'] = changed_inputs
        result['tool_passed'] = False
        result['timing_gate_passed'] = False
    artifacts = [syn, *required, log, synthesis_gate, out / 'timing_gate.json', out / 'timing_gate.log',
                 pnr / f'{NAME}_tr_content.html']
    result['artifacts'] = {p.relative_to(out).as_posix(): digest(p) for p in artifacts if p.is_file()}
    write_json(result_path, result)
    print(json.dumps({key: result[key] for key in ('elapsed_seconds', 'stage_seconds',
          'tool_passed', 'timing_gate_passed', 'incremental_fallback', 'pnr_timed_out')}, ensure_ascii=False), flush=True)
    if changed_inputs:
        raise SystemExit(f'构建期间输入发生变化：{changed_inputs}；本轮构建无效')
    if not result['tool_passed'] or (stage != 'syn' and not result['timing_gate_passed']):
        raise SystemExit(f'构建/时序检查未通过，证据：{out}')


if __name__ == '__main__':
    main()
