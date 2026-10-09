#!/usr/bin/env python3
"""Download a pinned ACT checkpoint and benchmark CPU inference. No robot I/O."""
import argparse
import hashlib
import importlib.metadata
import json
import platform
import statistics
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/act/smoke.json'))
    parser.add_argument('--iterations', type=int, default=10)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--image', type=Path, help='Optional saved wrist RGB image; resized to 640x480')
    parser.add_argument('--state', type=float, nargs=6, help='Source-policy units, never raw servo ticks')
    args = parser.parse_args()
    if args.iterations < 2 or args.warmup < 0 or args.threads < 1:
        parser.error('iterations >= 2, warmup >= 0 and threads >= 1 are required')

    import numpy as np
    import torch
    from PIL import Image
    from huggingface_hub import snapshot_download
    from safetensors.torch import load_file
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    source = json.loads(Path(__file__).with_name('source.json').read_text())
    version = importlib.metadata.version('lerobot')
    if version != source['lerobot_version']:
        raise RuntimeError(f"Expected LeRobot {source['lerobot_version']}, got {version}")
    torch.set_num_threads(args.threads)
    torch.manual_seed(0)
    root = Path(snapshot_download(source['repo_id'], revision=source['revision'],
                                  allow_patterns=source['files']))
    hashes = {}
    for name in source['files']:
        path = root / name
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        hashes[name] = digest.hexdigest()
    raw = json.loads((root / 'config.json').read_text())
    expected = {'observation.state': [6], 'observation.images.wrist': [3, 480, 640]}
    if raw['type'] != 'act' or {k: v['shape'] for k, v in raw['input_features'].items()} != expected:
        raise RuntimeError('Unexpected source input schema')
    if raw['output_features']['action']['shape'] != [6] or raw['chunk_size'] != 100:
        raise RuntimeError('Unexpected source action schema')
    cfg = PreTrainedConfig.from_pretrained(root)
    if not isinstance(cfg, ACTConfig):
        raise RuntimeError('Checkpoint did not load as ACTConfig')
    cfg.device = 'cpu'
    cfg.use_amp = False
    cfg.pretrained_path = root
    # The full checkpoint supplies backbone weights: no ImageNet download needed.
    cfg.pretrained_backbone_weights = None
    policy = ACTPolicy.from_pretrained(root, config=cfg, strict=True).eval()
    pre, post = make_pre_post_processors(
        cfg, pretrained_path=str(root),
        preprocessor_overrides={'device_processor': {'device': 'cpu'}},
        postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    stats = load_file(str(root / 'policy_preprocessor_step_3_normalizer_processor.safetensors'))
    state_mean = next(v for k, v in stats.items() if 'observation.state' in k and k.endswith('mean'))
    state = torch.tensor(args.state, dtype=torch.float32) if args.state else state_mean.flatten().float()
    if args.image:
        rgb = np.asarray(Image.open(args.image).convert('RGB').resize((640, 480))).copy()
        image = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255
    else:
        image = torch.full((3, 480, 640), 0.5)
    if not torch.isfinite(state).all():
        raise ValueError('State must be finite')
    observation = {'observation.state': state, 'observation.images.wrist': image}
    # Check that source mean/std state normalization was actually restored.
    mean_batch = pre({'observation.state': state_mean.flatten().float(),
                      'observation.images.wrist': image.clone()})
    if not torch.allclose(mean_batch['observation.state'],
                          torch.zeros_like(mean_batch['observation.state']), atol=1e-6):
        raise RuntimeError('Saved source state normalization did not restore correctly')
    timings = []
    with torch.inference_mode():
        for i in range(args.warmup + args.iterations):
            policy.reset()  # Always measure a fresh chunk, never a cached select_action.
            start = time.perf_counter()
            batch = pre({k: v.clone() for k, v in observation.items()})
            chunk = policy.predict_action_chunk(batch)
            actions = torch.stack([post(chunk[:, j].clone()) for j in range(chunk.shape[1])], dim=1)
            elapsed = time.perf_counter() - start
            if tuple(actions.shape) != (1, 100, 6) or not torch.isfinite(actions).all():
                raise RuntimeError('Invalid decoded action chunk')
            if i >= args.warmup:
                timings.append(elapsed * 1000)
    p95 = float(np.percentile(timings, 95))
    report = {
        'source': source, 'sha256': hashes,
        'software': {k: importlib.metadata.version(k) for k in ['lerobot', 'torch', 'torchvision']},
        'host': {'platform': platform.platform(), 'python': platform.python_version(), 'threads': args.threads},
        'input': {'image': str(args.image) if args.image else 'synthetic constant RGB',
                  'state': 'provided source units' if args.state else 'source normalization mean'},
        'output_shape': list(actions.shape), 'all_finite': True,
        'latency_ms': {'median': statistics.median(timings), 'p95': p95, 'samples': timings},
        'fresh_chunk_hz_at_p95': 1000 / p95,
        'control_fps': None,
        'action_semantics': 'PARTIAL: public card datasets confirm joint order and 30 FPS; train_config v2 metadata unavailable; units/gripper calibration unverified',
        'limitations': 'Command-free CPU smoke test. Synthetic inputs do not measure grasp quality. Control FPS requires sensor and executor measurements; chunk length is not permission to execute open-loop.'
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ['output_shape', 'latency_ms', 'fresh_chunk_hz_at_p95', 'control_fps']}, indent=2))
    print(f'Report: {args.output}')


if __name__ == '__main__':
    main()
