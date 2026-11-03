"""Check the installed entry point with YAML, without sending flight commands."""

import signal
import subprocess
import sys
import tempfile
from pathlib import Path


executable, config_dir, mavros_built = sys.argv[1:]


def run(config=None, running=False, extra_args=()):
    arguments = [executable, '--ros-args']
    if config is not None:
        arguments.extend(['--params-file', str(config)])
    arguments.extend(extra_args)
    process = subprocess.Popen(
        arguments,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        try:
            output, _ = process.communicate(timeout=3 if running else 10)
        except subprocess.TimeoutExpired:
            if not running:
                raise
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=10)
        else:
            assert not running, f'Control node exited before shutdown:\n{output}'
        assert process.returncode == (0 if running else 1), output
        return output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()


backend = Path(config_dir) / 'backend.yaml'
output = run(backend, running=True)
assert 'Selected backend: dds' in output and 'DDS READ ONLY: 20.0 Hz' in output, output
assert '/config/dds.yaml' in output, output
assert 'configure backend.yaml' in run()
with tempfile.TemporaryDirectory() as directory:
    invalid = Path(directory) / 'invalid.yaml'
    invalid.write_text(backend.read_text().replace('backend: dds', 'backend: unknown'))
    assert "backend must be 'dds' or 'mavros'" in run(invalid)
    dds = Path(config_dir) / 'dds.yaml'
    parameters = Path(directory) / 'dds.yaml'
    override = ['-p', f'backend_params_file:={parameters}']
    parameters.write_text(dds.read_text().replace('publish_rate_hz: 20.0', 'publish_rate_hz: 0.0'))
    assert 'DDS rates and timeouts must be finite and positive' in run(backend, extra_args=override)
    parameters.write_text(dds.read_text().replace('publish_rate_hz: 20.0', 'publish_rate_hz: 29.0'))
    assert 'DDS READ ONLY: 29.0 Hz' in run(backend, running=True, extra_args=override)
    assert 'DDS READ ONLY: 31.0 Hz' in run(
        backend, running=True, extra_args=[*override, '-p', 'publish_rate_hz:=31.0'])
    assert 'Error opening YAML file' in run(
        backend, extra_args=['-p', f'backend_params_file:={directory}/missing.yaml'])
    mavros = Path(directory) / 'backend.yaml'
    mavros.write_text(backend.read_text().replace('backend: dds', 'backend: mavros'))
    if mavros_built == 'ON':
        assert 'Selected backend: mavros' in run(mavros, running=True)
    else:
        assert 'MAVROS backend is not built' in run(mavros)
