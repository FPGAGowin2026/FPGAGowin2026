import signal
import subprocess
import tempfile


def test_default_launch_without_calibration_only_produces_invalid_candidate():
    log = tempfile.TemporaryFile(mode='w+t')
    process = subprocess.Popen(
        ['ros2', 'launch', 'swarm_bringup', 'handoff.launch.py'],
        stdout=log, stderr=subprocess.STDOUT)
    try:
        projection = subprocess.run(
            ['ros2', 'topic', 'echo', '--once', '/swarm/target_position_candidate',
             'swarm_interfaces/msg/TargetPositionCandidate'],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=12)
        received = subprocess.run(
            ['ros2', 'topic', 'echo', '--once', '/swarm/legged_target_candidate',
             'swarm_interfaces/msg/TargetPositionCandidate'],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=12)
        assert process.poll() is None
        log.flush()
        log.seek(0)
        assert (projection.returncode == 0 and 'valid: false' in projection.stdout and
                'reason: calibration_missing' in projection.stdout and
                received.returncode == 0 and 'valid: false' in received.stdout), (
                    f'{projection.stdout}\n{projection.stderr}\n'
                    f'{received.stdout}\n{received.stderr}\n{log.read()}')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
