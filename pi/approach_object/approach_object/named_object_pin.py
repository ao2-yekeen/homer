"""Screened checkpoint and workstation versions for named-object diagnostics."""

MODEL_SHA256 = '1741c1f8da3cea47e2c01829c334a50dc0b9bbd05e685b90a3ce84fae32c8c1b'
MODEL_URL = 'https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26n-seg.pt'
TEXT_ENCODER_SHA256 = '35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f'
TEXT_ENCODER_URL = 'https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts'
CLIP_REVISION = 'a13192f8cb767260d7dfd98c843b0716593169e7'
DEPENDENCIES = {
    'ultralytics': '8.4.173',
    'torch': '2.8.0+cpu',
    'torchvision': '0.23.0+cpu',
    'numpy': '1.26.4',
    'scipy': '1.15.3',
    'opencv-python': '4.11.0.86',
    'PyYAML': '6.0.2',
    'clip': '1.0',
    'ftfy': '6.3.1',
    'regex': '2026.9.29',
}


def ensure_asset(path, url, expected_sha256):
    """Download once; verify before allowing either Torch model to load."""
    import hashlib
    import shutil
    import urllib.request
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.is_file():
        partial = path.with_suffix('.download')
        with urllib.request.urlopen(url, timeout=60) as source, partial.open('wb') as destination:
            shutil.copyfileobj(source, destination)
        if hashlib.sha256(partial.read_bytes()).hexdigest() != expected_sha256:
            partial.unlink()
            raise ValueError('downloaded asset checksum differs from screened file: ' + path.name)
        partial.replace(path)
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
        raise ValueError('asset checksum differs from screened file: ' + path.name)
    return path
