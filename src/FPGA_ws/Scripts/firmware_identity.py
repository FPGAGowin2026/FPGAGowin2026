"""计算固件全部源文件的确定性身份，防止陈旧ROM进入新位流。"""
import hashlib
from pathlib import Path


def firmware_input_sha256(directory):
    directory = Path(directory)
    digest = hashlib.sha256()
    for path in sorted(p for p in directory.iterdir() if p.is_file()):
        name = path.name.encode('utf-8')
        data = path.read_bytes()
        digest.update(len(name).to_bytes(4, 'big'))
        digest.update(name)
        digest.update(len(data).to_bytes(8, 'big'))
        digest.update(data)
    return digest.hexdigest()
