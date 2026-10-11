#!/usr/bin/env python3
"""Train official LeRobot ACT directly from the recorder's JPEG/gzip episodes."""
import argparse
import bisect
import gzip
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset
from torch.utils.data import default_collate

from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.processor_act import make_act_pre_post_processors
from lerobot.policies.factory import make_pre_post_processors

CAMERAS = {"observation.images.wrist": "image",
           "observation.images.overview": "overview_image"}
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def load_episode(directory):
    metadata = json.loads((directory / "metadata.json").read_text())
    if metadata.get("outcome") != "success" or metadata.get("joint_only"):
        raise ValueError(f"Not a successful camera demo: {directory}")
    if metadata["config"]["joint_names"] != JOINTS:
        raise ValueError(f"Different joint order: {directory}")
    compressed = directory / "samples.jsonl.gz"
    with (gzip.open(compressed, "rt") if compressed.exists()
          else (directory / "samples.jsonl").open()) as stream:
        rows = [json.loads(line) for line in stream]
    ticks = np.asarray([row["state_ticks"] for row in rows], dtype=np.float32)
    if len(rows) < 2 or len(rows) != metadata["samples"] or ticks.shape != (len(rows), 6):
        raise ValueError(f"Incomplete joint samples: {directory}")
    if not np.isfinite(ticks).all():
        raise ValueError(f"Nonfinite joint samples: {directory}")
    for row in rows:
        for field in CAMERAS.values():
            if not (directory / row[field]).is_file():
                raise ValueError(f"Missing {field}: {directory}")
    return directory, rows, ticks


class DemoDataset(Dataset):
    def __init__(self, episodes, chunk_size=50, image_size=(160, 120)):
        self.episodes = episodes
        self.chunk_size = chunk_size
        self.image_size = image_size
        self.ends = np.cumsum([len(rows) - 1 for _, rows, _ in episodes]).tolist()

    def __len__(self):
        return self.ends[-1]

    def __getitem__(self, index):
        episode = bisect.bisect_right(self.ends, index)
        frame = index - (self.ends[episode - 1] if episode else 0)
        directory, rows, ticks = self.episodes[episode]
        # At frame t, predict t+1 onward. Clamp and mask at this episode's end.
        future = np.arange(frame + 1, frame + 1 + self.chunk_size)
        sample = {"observation.state": torch.from_numpy(ticks[frame].copy()),
                  "action": torch.from_numpy(ticks[np.minimum(future, len(rows) - 1)].copy()),
                  "action_is_pad": torch.from_numpy(future >= len(rows))}
        for key, field in CAMERAS.items():
            with Image.open(directory / rows[frame][field]) as image:
                rgb = np.array(image.convert("RGB").resize(self.image_size, Image.Resampling.BILINEAR))
            sample[key] = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255
        return sample


def training_stats(episodes):
    stats = {}
    for key, offset in (("observation.state", 0), ("action", 1)):
        values = np.concatenate([ticks[:-1] if offset == 0 else ticks[1:]
                                 for _, _, ticks in episodes])
        stats[key] = {"mean": torch.from_numpy(values.mean(0)),
                      "std": torch.from_numpy(values.std(0).clip(min=1.0))}
    for key in CAMERAS:
        stats[key] = {"mean": torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1),
                      "std": torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)}
    return stats


def sample_indices(dataset, manifest, count):
    """Replay old demos while giving corrections an explicit sampling share."""
    corrections = set(manifest.get("correction_train", []))
    if not corrections:
        return [random.randrange(len(dataset)) for _ in range(count)]
    groups = {False: [], True: []}
    for i, (directory, _, _) in enumerate(dataset.episodes):
        begin = dataset.ends[i-1] if i else 0
        groups[directory.name in corrections].extend(range(begin, dataset.ends[i]))
    return [random.choice(groups[random.random() < manifest.get("correction_fraction", 0.5)])
            for _ in range(count)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("act/data"))
    parser.add_argument("--episodes-file", type=Path)
    parser.add_argument("--output", type=Path, default=Path("act/outputs/act-pi-demos"))
    parser.add_argument("--steps", type=int, default=20000)
    parser.add_argument("--save-every", type=int, default=200)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--image-width", type=int, default=320)
    parser.add_argument("--image-height", type=int, default=240)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dataset-manifest", type=Path,
                        help="Explicit episode split, including correction sampling groups")
    parser.add_argument("--init-from", type=Path, help="Fine-tune these weights and saved normalizers")
    parser.add_argument("--learning-rate", type=float)
    args = parser.parse_args()
    torch.set_num_threads(args.threads)
    torch.manual_seed(42)
    random.seed(42)
    np.random.seed(42)
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output / "dataset.json"
    if args.resume:
        manifest = json.loads(manifest_path.read_text())
    else:
        if (args.output / "latest").exists():
            parser.error("Output already has a checkpoint; use --resume or another --output")
        names = ([line.strip().rstrip("/") for line in args.episodes_file.read_text().splitlines()]
                 if args.episodes_file else
                 [p.name for p in sorted(args.data.glob("episode-*"))
                  if (p / "metadata.json").exists()
                  and json.loads((p / "metadata.json").read_text()).get("outcome") == "success"])
        random.Random(42).shuffle(names)
        n_val = max(1, round(len(names) * 0.1))
        if len(names) < 3:
            parser.error("Need at least three successful episodes for train/validation")
        manifest = {"train": names[n_val:], "validation": names[:n_val],
                    "joint_names": JOINTS, "state_units": "raw servo ticks",
                    "action_offset_frames": 1, "chunk_size": 50,
                    "image_size": [args.image_width, args.image_height], "fps": 10, "seed": 42}
        if args.dataset_manifest:
            manifest = json.loads(args.dataset_manifest.read_text())
        if args.init_from:
            manifest['init_from'] = str(args.init_from)
    if args.init_from and args.resume:
        parser.error("Use --resume alone after the initial fine-tuning run")
    train_episodes = [load_episode(args.data / name) for name in manifest["train"]]
    val_episodes = [load_episode(args.data / name) for name in manifest["validation"]]
    train = DemoDataset(train_episodes, manifest["chunk_size"], tuple(manifest["image_size"]))
    val = DemoDataset(val_episodes, manifest["chunk_size"], tuple(manifest["image_size"]))
    manifest.update(train_frames=len(train), validation_frames=len(val))
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Dataset: {len(train_episodes)} train demos ({len(train)} frames), "
          f"{len(val_episodes)} validation demos ({len(val)} frames)", flush=True)
    stats = training_stats(train_episodes)
    w, h = manifest["image_size"]
    config = ACTConfig(device=args.device, push_to_hub=False, chunk_size=manifest["chunk_size"],
                       n_action_steps=manifest["chunk_size"],
                       input_features={"observation.state": PolicyFeature(FeatureType.STATE, (6,)),
                                       **{key: PolicyFeature(FeatureType.VISUAL, (3, h, w)) for key in CAMERAS}},
                       output_features={"action": PolicyFeature(FeatureType.ACTION, (6,))})
    latest = args.output / "latest"
    initialization = manifest.get('init_from')
    if initialization:
        source = latest if args.resume else Path(initialization)
        config = PreTrainedConfig.from_pretrained(source)
        config.device = args.device
        config.pretrained_backbone_weights = None
        policy = ACTPolicy.from_pretrained(source, config=config, strict=True)
        pre, post = make_pre_post_processors(
            config, pretrained_path=str(source),
            preprocessor_overrides={'device_processor': {'device': args.device}},
            postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    elif args.resume:
        config.pretrained_backbone_weights = None
        policy = ACTPolicy.from_pretrained(latest, config=config, strict=True)
    else:
        policy = ACTPolicy(config)
    policy.to(args.device)
    if not initialization:
        pre, post = make_act_pre_post_processors(config, stats)
    learning_rate = args.learning_rate or (1e-5 if initialization else config.optimizer_lr)
    if initialization:
        config.optimizer_lr_backbone = learning_rate
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=learning_rate,
                                  weight_decay=config.optimizer_weight_decay, foreach=False)
    step = 0
    best_mae = float("inf")
    if args.resume:
        saved = torch.load(latest / "training.pt", map_location="cpu", weights_only=False)
        optimizer.load_state_dict(saved["optimizer"])
        step = saved["step"]
        best_mae = saved.get("best_mae", float("inf"))
        torch.set_rng_state(saved["torch_rng"])
        if args.device == "cuda" and "cuda_rng" in saved:
            torch.cuda.set_rng_state_all(saved["cuda_rng"])
        random.setstate(saved["python_rng"])
        del saved
        print(f"Resumed optimizer and policy at step {step}", flush=True)

    def batch(dataset, indices):
        return pre(default_collate([dataset[index] for index in indices]))

    def validate():
        policy.eval()
        errors = {}
        correction_names = set(manifest.get('correction_validation', []))
        with torch.inference_mode():
            # Equal weight per episode, then per original/correction group.
            for episode, (directory, rows, _) in enumerate(val.episodes):
                begin = val.ends[episode-1] if episode else 0
                episode_errors = []
                for frame in np.linspace(0, len(rows)-2, min(4, len(rows)-1), dtype=int):
                    sample = batch(val, [begin+int(frame)])
                    prediction = policy.predict_action_chunk(sample)
                    valid = ~sample['action_is_pad'].cpu()
                    difference = (post(prediction.clone()) - post(sample['action'].clone())).abs()
                    episode_errors.append(difference[valid].mean().item())
                group = 'correction' if directory.name in correction_names else 'original'
                errors.setdefault(group, []).append(float(np.mean(episode_errors)))
        policy.train()
        return {group: float(np.mean(values)) for group, values in errors.items()}

    def save():
        # Keep one resumable checkpoint rather than accumulating large optimizer files.
        staging = args.output / "checkpoint-writing"
        policy.save_pretrained(staging)
        pre.save_pretrained(staging)
        post.save_pretrained(staging)
        torch.save({"step": step, "optimizer": optimizer.state_dict(), "best_mae": best_mae,
                    "cuda_rng": torch.cuda.get_rng_state_all() if args.device == "cuda" else [],
                    "torch_rng": torch.get_rng_state(), "python_rng": random.getstate()},
                   staging / "training.pt")
        (staging / "dataset.json").write_text(manifest_path.read_text())
        import shutil
        previous = args.output / "checkpoint-previous"
        if previous.exists():
            shutil.rmtree(previous)
        if latest.exists():
            latest.rename(previous)
        staging.rename(latest)
        if previous.exists():
            shutil.rmtree(previous)
        print(f"Checkpoint saved: {latest} (step {step})", flush=True)

    policy.train()
    (args.output / "run.json").write_text(json.dumps({
        "steps": args.steps, "batch_size": args.batch_size, "device": args.device,
        "torch": torch.__version__, "lerobot": "0.4.4",
        "gpu": torch.cuda.get_device_name(0) if args.device == "cuda" else None,
        "learning_rate": learning_rate, "init_from": initialization,
        "correction_fraction": manifest.get('correction_fraction')}, indent=2) + "\n")
    start = time.monotonic()
    initial_step = step
    with (args.output / "metrics.jsonl").open("a", buffering=1) as log:
        try:
            while step < args.steps:
                sample = batch(train, sample_indices(train, manifest, args.batch_size))
                optimizer.zero_grad(set_to_none=True)
                loss, losses = policy(sample)
                if not torch.isfinite(loss):
                    raise RuntimeError(f"Nonfinite training loss at step {step}")
                loss.backward()
                torch.nn.utils.clip_grad_norm_(policy.parameters(), 10, error_if_nonfinite=True)
                optimizer.step()
                step += 1
                if step == initial_step + 1 or step % 10 == 0:
                    metric = {"step": step, "loss": loss.item(), **losses,
                              "seconds_per_step": (time.monotonic() - start) / (step - initial_step)}
                    log.write(json.dumps(metric) + "\n")
                    remaining = metric["seconds_per_step"] * (args.steps - step)
                    print(f"Step {step:,}/{args.steps:,} ({step / args.steps:.1%}) | "
                          f"loss {loss.item():.4f} | {metric['seconds_per_step']:.2f} s/step | "
                          f"ETA {remaining / 3600:.1f} hours", flush=True)
                if step % args.save_every == 0 or step == args.steps:
                    group_mae = validate()
                    mae = float(np.mean(list(group_mae.values())))
                    log.write(json.dumps({"step": step, "validation_mae_ticks": mae,
                                          "validation_groups": group_mae}) + "\n")
                    print(f"Validation mean absolute error: {mae:.2f} ticks; {group_mae}", flush=True)
                    if mae < best_mae:
                        best_mae = mae
                        best = args.output / "best"
                        policy.save_pretrained(best)
                        pre.save_pretrained(best)
                        post.save_pretrained(best)
                        (best / "dataset.json").write_text(manifest_path.read_text())
                        (best / "validation.json").write_text(json.dumps({"step": step, "mae_ticks": mae}))
                    save()
        except KeyboardInterrupt:
            print("Interrupted; saving training state.", flush=True)
            save()


if __name__ == "__main__":
    main()
