import hashlib
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
import pytest
from approach_object import model_io

@pytest.fixture
def model(tmp_path, monkeypatch):
    hashes = {}
    for name in ('metadata.yaml', 'model.ncnn.bin', 'model.ncnn.param'):
        (tmp_path/name).write_bytes(b'model')
        hashes[name] = hashlib.sha256(b'model').hexdigest()
    manifest = dict(model='yolo11n-seg', imgsz=320, sha256=hashes, source_sha256='source', dependencies={p:'pinned' for p in ('ultralytics','ncnn','numpy','opencv-python')})
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    monkeypatch.setattr(model_io, 'MODEL_SHA256', {k:v for k,v in hashes.items() if k != 'metadata.yaml'})
    monkeypatch.setattr(model_io, 'SOURCE_SHA256', 'source')
    monkeypatch.setattr(model_io.md, 'version', lambda p: 'pinned')
    return tmp_path

def test_reviewed_model_loads(model):
    assert model_io.verify_model(model)['imgsz'] == 320

def test_changed_weight_rejected(model):
    (model/'model.ncnn.bin').write_bytes(b'changed')
    with pytest.raises(ValueError, match='checksum'): model_io.verify_model(model)

def test_unreviewed_manifest_rejected(model):
    manifest = json.loads((model/'manifest.json').read_text())
    manifest['source_sha256'] = 'different source'
    (model/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='revision'): model_io.verify_model(model)

def test_unpinned_dependency_rejected(model, monkeypatch):
    monkeypatch.setattr(model_io.md, 'version', lambda p: 'different')
    with pytest.raises(ValueError, match='dependency'): model_io.verify_model(model)
