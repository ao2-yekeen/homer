#!/usr/bin/env python3
"""Offline ACT camera ablation; writes predictions without connecting to hardware."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors


def image_tensor(path, size):
    with Image.open(path) as image:
        rgb = np.array(image.convert('RGB').resize(size, Image.Resampling.BILINEAR))
    return torch.from_numpy(rgb).permute(2, 0, 1).float() / 255


def metrics(actions, baseline):
    delta = np.abs(actions - baseline)
    return {
        'first_action_mean_abs_ticks': float(delta[0].mean()),
        'first_five_mean_abs_ticks': float(delta[:5].mean()),
        'chunk_mean_abs_ticks': float(delta.mean()),
        'chunk_max_abs_ticks': float(delta.max()),
        'per_joint_mean_abs_ticks': delta.mean(axis=0).tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--data', type=Path, default=Path('act/data'))
    parser.add_argument('--frames', type=int, nargs='+', default=[0, 3, 6, 8])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.checkpoint / 'dataset.json').read_text())
    donors = []
    for episode in manifest['validation']:
        folder = args.data / episode
        compressed = folder / 'samples.jsonl.gz'
        opener, path = (gzip.open, compressed) if compressed.exists() else (open, folder / 'samples.jsonl')
        with opener(path, 'rt') as stream:
            for index, line in enumerate(stream):
                row = json.loads(line)
                donors.append((folder, index, row))
    donor_states = np.array([row['state_ticks'] for _, _, row in donors])
    torch.set_num_threads(2)
    config = PreTrainedConfig.from_pretrained(args.checkpoint)
    config.device = 'cpu'
    config.pretrained_backbone_weights = None
    policy = ACTPolicy.from_pretrained(args.checkpoint, config=config, strict=True).to('cpu').eval()
    pre, post = make_pre_post_processors(
        config, pretrained_path=str(args.checkpoint),
        preprocessor_overrides={'device_processor': {'device': 'cpu'}},
        postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    size = tuple(manifest['image_size'])

    def predict(state, images):
        observation = {'observation.state': torch.tensor(state, dtype=torch.float32)}
        observation.update({f'observation.images.{name}': value for name, value in images.items()})
        # Reset processors too so repeated inputs cannot inherit temporal state.
        policy.reset()
        pre.reset()
        post.reset()
        with torch.inference_mode():
            chunk = policy.predict_action_chunk(pre(observation))
            decoded = torch.stack([post(chunk[:, i].clone()) for i in range(chunk.shape[1])], dim=1)[0]
        return decoded.numpy()

    checkpoint_hash = hashlib.sha256()
    with (args.checkpoint / 'model.safetensors').open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            checkpoint_hash.update(block)
    report = {
        'checkpoint': str(args.checkpoint),
        'checkpoint_sha256': checkpoint_hash.hexdigest(),
        'joint_names': manifest['joint_names'],
        'image_size': manifest['image_size'],
        'torch_version': torch.__version__,
        'limitations': 'Image influence does not establish correct localisation or task success. '
                        'Donor images can mismatch the fixed joint state; blank images are out of distribution.',
        'anchors': [],
    }
    for frame in args.frames:
        prefix = args.run / f'{frame:03d}'
        state_record = json.loads(Path(f'{prefix}-state.json').read_text())
        if state_record['joint_names'] != manifest['joint_names']:
            raise ValueError('Snapshot and checkpoint joint orders differ')
        state = state_record['state_ticks']
        distances = np.sqrt(np.mean((donor_states - state) ** 2, axis=1))
        donor_id = int(distances.argmin())
        folder, index, row = donors[donor_id]
        base_paths = {name: Path(f'{prefix}-{name}.jpg') for name in ('wrist', 'overview')}
        donor_paths = {'wrist': folder / row['image'], 'overview': folder / row['overview_image']}
        base = {name: image_tensor(path, size) for name, path in base_paths.items()}
        replacement = {name: image_tensor(path, size) for name, path in donor_paths.items()}
        cases = {'baseline': base, 'identical_repeat': base}
        for label, images in [('donor', replacement), ('blank', {n: torch.full_like(t, 0.5) for n, t in base.items()})]:
            for names in [('wrist',), ('overview',), ('wrist', 'overview')]:
                cases[f'{label}_{"_".join(names)}'] = {**base, **{name: images[name] for name in names}}
        predictions = {name: predict(state, images) for name, images in cases.items()}
        comparisons = {name: metrics(actions, predictions['baseline']) for name, actions in predictions.items()}
        report['anchors'].append({
            'frame': frame, 'state_ticks': state, 'base_images': {k: str(v) for k, v in base_paths.items()},
            'donor_episode': folder.name, 'donor_row': index, 'donor_state_ticks': row['state_ticks'],
            'donor_joint_rms_ticks': float(distances[donor_id]),
            'donor_images': {k: str(v) for k, v in donor_paths.items()},
            'actions': {name: value.tolist() for name, value in predictions.items()}, 'comparisons': comparisons,
        })
        print(f'Frame {frame}, donor joint RMS distance {distances[donor_id]:.1f} ticks', flush=True)
        for name, values in comparisons.items():
            print(f'  {name}: first five={values["first_five_mean_abs_ticks"]:.3f}, '
                  f'full chunk={values["chunk_mean_abs_ticks"]:.3f} ticks', flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'Saved {args.output}', flush=True)


if __name__ == '__main__':
    main()
