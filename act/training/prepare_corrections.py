#!/usr/bin/env python3
"""Prepare compressed corrections and original demos without editing recordings."""
import argparse
from copy import deepcopy
import gzip
import json
import math
from pathlib import Path
import random
import shutil

import numpy as np
from PIL import Image

CAMERAS = ('image', 'overview_image')


def read_rows(directory):
    compressed = directory / 'samples.jsonl.gz'
    with (gzip.open(compressed, 'rt') if compressed.exists()
          else (directory / 'samples.jsonl').open()) as stream:
        return [json.loads(line) for line in stream]


def visual_motion(a, b):
    """Conservative image-change heuristic; not an object motion detector."""
    difference = np.abs(a.astype(np.int16) - b.astype(np.int16))
    return difference.mean() > 2.5 or (difference > 20).mean() > 0.02


def retained_indices(rows, views, fps=10, hold_seconds=0.5):
    """Cap long quiet runs, preserving bounded drift, visual motion and grip context.

    Compare to the run's first pose/image as well as adjacent frames, so slow
    motion cannot disappear through a series of individually small changes.
    """
    ticks = np.asarray([row['state_ticks'] for row in rows])
    protected = np.zeros(len(rows), dtype=bool)
    context = math.ceil(0.5 * fps)
    for index in np.flatnonzero(np.abs(np.diff(ticks[:, -1])) > 2) + 1:
        protected[max(0, index-context):min(len(rows), index+context+1)] = True
    keep_count = max(2, math.ceil(hold_seconds * fps))
    minimum_run = max(keep_count + 1, math.ceil(fps))
    keep = []
    runs = []
    start = 0
    while start < len(rows):
        end = start + 1
        if not protected[start]:
            while end < len(rows) and not protected[end]:
                if (np.max(np.abs(ticks[end] - ticks[start])) > 3
                        or np.max(np.abs(ticks[end] - ticks[end-1])) > 3):
                    break
                if any(visual_motion(view[end], view[start])
                       or visual_motion(view[end], view[end-1]) for view in views):
                    break
                end += 1
        if end - start >= minimum_run:
            before = (keep_count + 1) // 2
            after = keep_count // 2
            keep.extend(range(start, start+before))
            keep.extend(range(end-after, end))
            runs.append([start+before, end-after])
        else:
            keep.extend(range(start, end))
        start = end
    return keep, runs


def export_episode(source, destination, metadata, rows, indices, image_size):
    destination.mkdir(parents=True)
    exported = []
    for original_index in indices:
        row = deepcopy(rows[original_index])
        row['source_frame'] = original_index
        for field in CAMERAS:
            image_path = source / row[field]
            target = Path(row[field]).with_suffix('.jpg').name
            with Image.open(image_path) as image:
                if image.format == 'JPEG' and image.size == image_size:
                    shutil.copyfile(image_path, destination / target)
                else:
                    image.convert('RGB').resize(image_size, Image.Resampling.BILINEAR).save(
                        destination / target, quality=75)
            row[field] = target
        exported.append(row)
    metadata = deepcopy(metadata)
    metadata['samples'] = len(exported)
    metadata['config'].update(width=image_size[0], height=image_size[1])
    metadata['config']['overview_camera'].update(width=image_size[0], height=image_size[1])
    metadata['training_export'] = {
        'source_episode': source.name, 'original_samples': len(rows),
        'image_size': list(image_size), 'jpeg_quality': 75,
        'timestamps': 'Original acquisition timestamps preserved; train at nominal 10 FPS',
        'training_duration_seconds': (len(exported)-1)/10,
    }
    with gzip.open(destination / 'samples.jsonl.gz', 'wt') as stream:
        for row in exported:
            stream.write(json.dumps(row, separators=(',', ':')) + '\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('act/data'))
    parser.add_argument('--checkpoint', type=Path, default=Path('act/outputs/kaggle-trained/policy'))
    parser.add_argument('--output', type=Path, default=Path('act/outputs/correction-training'))
    parser.add_argument('--resume', action='store_true', help='Continue an interrupted export')
    args = parser.parse_args()
    # Refuse to overwrite a prepared dataset or the original recordings.
    if args.output.resolve().is_relative_to(args.data.resolve()):
        parser.error('Training copies must be outside the original recordings directory')
    args.output.mkdir(parents=True, exist_ok=args.resume)
    original = json.loads((args.checkpoint / 'dataset.json').read_text())
    corrections = [p.name for p in sorted(args.data.glob('episode-*'))
                   if (p / 'metadata.json').exists()
                   and (m := json.loads((p / 'metadata.json').read_text())).get('outcome') == 'success'
                   and (m.get('assistance') or {}).get('mode') == 'manual-correction']
    random.Random(42).shuffle(corrections)
    n_val = max(1, round(len(corrections)*0.1))
    manifest = deepcopy(original)
    manifest.update(train=original['train'] + corrections[n_val:],
                    validation=original['validation'] + corrections[:n_val],
                    correction_train=corrections[n_val:], correction_validation=corrections[:n_val],
                    original_train=original['train'], original_validation=original['validation'],
                    init_from=str(args.checkpoint), correction_fraction=0.5)
    # Counts are recomputed by the trainer after loading the prepared episodes.
    manifest.pop('train_frames', None)
    manifest.pop('validation_frames', None)
    manifest_path = args.output / 'dataset.json'
    if args.resume and manifest_path.exists():
        if json.loads(manifest_path.read_text()) != manifest:
            parser.error('The source episode selection changed; use a new output directory')
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    report = []
    for name in manifest['train'] + manifest['validation']:
        source = args.data / name
        metadata = json.loads((source / 'metadata.json').read_text())
        rows = read_rows(source)
        correction = name in corrections
        destination = args.output / 'data' / name
        completed = destination / 'metadata.json'
        if args.resume and completed.exists():
            updated = json.loads(completed.read_text())
            export = updated['training_export']
            if export['source_episode'] != name or export['original_samples'] != len(rows):
                raise ValueError(f'Source recording changed: {name}; use a new output directory')
            trim = updated.get('pause_trim', {})
            report.append(dict(episode=name, correction=correction, original_frames=len(rows),
                               retained_frames=updated['samples'],
                               removed_frames=len(rows)-updated['samples'],
                               removed_ranges=trim.get('removed_ranges', [])))
            print(f'{name}: already exported', flush=True)
            continue
        if args.resume and destination.exists():
            # This directory belongs to the interrupted export, never the source.
            shutil.rmtree(destination)
        runs = []
        indices = list(range(len(rows)))
        if correction:
            views = []
            for field in CAMERAS:
                frames = []
                for row in rows:
                    with Image.open(source / row[field]) as image:
                        frames.append(np.asarray(image.convert('L').resize((80, 60))))
                views.append(np.stack(frames))
            indices, runs = retained_indices(rows, views, fps=manifest['fps'])
        updated = export_episode(source, destination, metadata, rows, indices, tuple(manifest['image_size']))
        if correction:
            updated['pause_trim'] = dict(
                version=1, hold_seconds=0.5, minimum_pause_seconds=1.0,
                joint_anchor_tolerance_ticks=3, gripper_context_seconds=0.5,
                visual_mean_difference=2.5, visual_changed_fraction=0.02,
                visual_pixel_difference=20, visual_size=[80, 60],
                removed_ranges=runs, range_end_exclusive=True,
                removed_frames=len(rows)-len(indices),
                limitation='Image change is a heuristic, not object/contact/settling detection')
        (destination / 'metadata.json').write_text(json.dumps(updated, indent=2)+'\n')
        report.append(dict(episode=name, correction=correction, original_frames=len(rows),
                           retained_frames=len(indices), removed_frames=len(rows)-len(indices),
                           removed_ranges=runs))
        print(f'{name}: {len(rows)} -> {len(indices)} frames', flush=True)
    (args.output / 'preparation.json').write_text(json.dumps(report, indent=2)+'\n')
    removed = sum(r['removed_frames'] for r in report)
    print(f'Ready: {args.output}; {len(corrections)} corrections, {removed} quiet frames removed. Originals untouched.')


if __name__ == '__main__':
    main()
