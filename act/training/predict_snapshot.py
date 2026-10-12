#!/usr/bin/env python3
"""Predict a decoded ACT action chunk from capture_policy_input.py output."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors


class SnapshotPolicy:
    def __init__(self, checkpoint):
        torch.set_num_threads(2)
        config = PreTrainedConfig.from_pretrained(checkpoint)
        config.device = 'cpu'
        config.pretrained_backbone_weights = None
        self.policy = ACTPolicy.from_pretrained(checkpoint, config=config, strict=True).to('cpu').eval()
        self.pre, self.post = make_pre_post_processors(
            config, pretrained_path=str(checkpoint),
            preprocessor_overrides={'device_processor': {'device': 'cpu'}},
            postprocessor_overrides={'device_processor': {'device': 'cpu'}})
        self.manifest = json.loads((checkpoint / 'dataset.json').read_text())

    def predict(self, state_ticks, images):
        observation = {'observation.state': torch.tensor(state_ticks, dtype=torch.float32)}
        for name, image in images.items():
            rgb = np.array(image.convert('RGB').resize(tuple(self.manifest['image_size']),
                                                       Image.Resampling.BILINEAR))
            observation[f'observation.images.{name}'] = torch.from_numpy(rgb).permute(2, 0, 1).float()/255
        with torch.inference_mode():
            start = time.perf_counter()
            prediction = self.policy.predict_action_chunk(self.pre(observation))
            actions = torch.stack([self.post(prediction[:, i].clone())
                                   for i in range(prediction.shape[1])], dim=1)[0]
        if not torch.isfinite(actions).all():
            raise RuntimeError('Policy produced nonfinite motor targets')
        return dict(actions=actions.tolist(), fps=self.manifest['fps'],
                    joint_names=self.manifest['joint_names'], state_ticks=state_ticks,
                    inference_seconds=time.perf_counter()-start)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    policy = SnapshotPolicy(args.checkpoint)
    state = json.loads((args.snapshot / 'state.json').read_text())
    images = {name: Image.open(args.snapshot / f'{name}.jpg') for name in ('wrist', 'overview')}
    try:
        result = policy.predict(state['state_ticks'], images)
    finally:
        for image in images.values():
            image.close()
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(f'Saved {len(result["actions"])} decoded actions to {args.output}', flush=True)


if __name__ == '__main__':
    main()
