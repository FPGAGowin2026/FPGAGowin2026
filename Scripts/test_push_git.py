"""使用临时本地远端验证推送，不访问 GitHub。"""

from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    script = Path(__file__).with_name("push_git.sh")
    subprocess.run(["bash", "-n", str(script)], check=True)
    with tempfile.TemporaryDirectory(prefix="push-git-") as directory:
        root = Path(directory)
        (root / "Scripts").mkdir()
        shutil.copy2(script, root / "Scripts" / script.name)

        def git(*args):
            return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()

        def push(target, message="自动提交测试"):
            return subprocess.run(
                ["bash", str(root / "Scripts" / script.name), target, message],
                capture_output=True, text=True,
            )

        git("init", "-b", "test-branch")
        git("config", "user.name", "Script Test")
        git("config", "user.email", "test@example.invalid")
        git("config", "commit.gpgsign", "false")
        git("-c", "commit.gpgsign=false", "commit", "--allow-empty", "-m", "initial")
        git("init", "--bare", "remote.git")
        git("remote", "add", "origin", str(root / "remote.git"))
        (root / ".gitignore").write_text("/remote.git/\n/ros_ws/\n/llm_ws/\n/ignored.txt\n")
        (root / "ignored.txt").write_text("不提交")
        marker = root / "uncommitted.txt"
        marker.write_text("keep local")
        assert push("main", " ").returncode != 0
        assert not git("diff", "--cached", "--name-only")
        result = push("main")
        assert result.returncode == 0, result.stderr
        assert git("--git-dir=remote.git", "rev-parse", "refs/heads/test-branch") == git("rev-parse", "HEAD")
        assert not git("status", "--porcelain")
        assert git("log", "-1", "--format=%s") == "自动提交测试"
        assert not git("ls-files", "ignored.txt")
        assert marker.read_text() == "keep local"
        committed = git("rev-parse", "HEAD")
        assert push("main").returncode == 0
        assert git("rev-parse", "HEAD") == committed
        marker.unlink()
        assert push("main", "删除文件").returncode == 0
        assert not git("ls-files", "uncommitted.txt")
        (root / "ros_ws").mkdir()
        assert push("ros_ws").returncode != 0
        assert push("unknown").returncode != 0
        (root / "llm_ws").mkdir()
        assert push("llm_ws").returncode != 0
        git("-C", "llm_ws", "init", "-b", "voice-test")
        git("-C", "llm_ws", "config", "user.name", "Script Test")
        git("-C", "llm_ws", "config", "user.email", "test@example.invalid")
        git("-C", "llm_ws", "config", "commit.gpgsign", "false")
        git("-C", "llm_ws", "remote", "add", "origin", str(root / "remote.git"))
        (root / "llm_ws" / "voice.txt").write_text("语音导航")
        parent_head = git("rev-parse", "HEAD")
        result = push("llm_ws", "更新语音导航")
        assert result.returncode == 0, result.stderr
        assert git("--git-dir=remote.git", "rev-parse", "refs/heads/voice-test") == git("-C", "llm_ws", "rev-parse", "HEAD")
        assert git("rev-parse", "HEAD") == parent_head
        assert not git("ls-files", "llm_ws")
        assert not git("-C", "llm_ws", "status", "--porcelain")
        git("checkout", "--detach")
        assert push("main").returncode != 0
    print("PASS: 自动暂存提交推送、忽略规则、删除文件、无改动重试、llm_ws 独立推送及无效输入拒绝")


if __name__ == "__main__":
    main()
