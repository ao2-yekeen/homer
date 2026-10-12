"""Remove only the stationary pause around replay-to-hand handover."""
from copy import deepcopy
import math


def trim_handover(rows, metadata):
    """Return aligned retained samples, updated metadata, and removed samples.

    Eight ticks allows feedback jitter. Keep 0.3 seconds before motion resumes
    so gradual starts survive; require three outside readings to ignore a spike.
    Original image references, ticks and capture timestamps remain untouched.
    """
    metadata = deepcopy(metadata)
    assistance = metadata.get('assistance') or {}
    boundary = metadata.get('approach_end_frame')
    if (metadata.get('handover_trim') or metadata.get('joint_only')
            or metadata.get('outcome') != 'success'
            or assistance.get('mode') != 'replay-then-hand'
            or not isinstance(boundary, int) or not 0 <= boundary < len(rows)):
        return rows, metadata, []

    reference = rows[boundary]['state_ticks']
    fps = metadata['config'].get('fps', 10)
    tolerance = 8
    context = max(1, math.ceil(0.3 * fps))

    def stationary(index):
        return all(abs(a-b) <= tolerance
                   for a, b in zip(rows[index]['state_ticks'], reference))

    start = boundary
    while start > 0 and stationary(start-1):
        start -= 1
    end = len(rows)
    for index in range(boundary+1, len(rows)):
        # A short feedback spike is not evidence that manual motion began.
        if (index+3 <= len(rows)
                and all(not stationary(i) for i in range(index, index+3))):
            end = index
            break
    cut_end = max(start, end-context)
    if cut_end-start < math.ceil(fps):
        return rows, metadata, []

    kept = rows[:start] + rows[cut_end:]
    removed = rows[start:cut_end]
    metadata['samples'] = len(kept)
    metadata['approach_end_frame'] = (boundary if boundary < start
                                      else max(start, boundary-len(removed)))
    metadata['handover_trim'] = dict(version=1, original_samples=len(rows),
                                    removed_frames=len(removed),
                                    removed_range=[start, cut_end],
                                    range_end_exclusive=True,
                                    tolerance_ticks=tolerance,
                                    kept_context_frames=context,
                                    timestamps='Original capture timestamps; removed pause leaves a gap')
    return kept, metadata, removed


def trim_dataset(data):
    """Apply handover trimming to completed episodes already on disk."""
    import gzip
    import json
    from pathlib import Path

    episodes = frames = 0
    for metadata_path in sorted(Path(data).glob('*/metadata.json')):
        directory = metadata_path.parent
        metadata = json.loads(metadata_path.read_text())
        if (metadata.get('outcome') != 'success' or metadata.get('joint_only')
                or metadata.get('handover_trim')
                or (metadata.get('assistance') or {}).get('mode') != 'replay-then-hand'):
            continue
        path = directory / 'samples.jsonl.gz'
        compressed = path.exists()
        if not compressed:
            path = directory / 'samples.jsonl'
        with (gzip.open(path, 'rt') if compressed else path.open()) as stream:
            rows = [json.loads(line) for line in stream]
        kept, updated, removed = trim_handover(rows, metadata)
        if not removed:
            continue
        sample_temporary = path.with_name(path.name+'.trimmed')
        metadata_temporary = metadata_path.with_name('metadata.trimmed.json')
        with (gzip.open(sample_temporary, 'wt') if compressed
              else sample_temporary.open('w')) as stream:
            for row in kept:
                stream.write(json.dumps(row)+'\n')
        metadata_temporary.write_text(json.dumps(updated, indent=2))
        sample_temporary.replace(path)
        metadata_temporary.replace(metadata_path)
        retained_images = {row[field] for row in kept
                           for field in ('image', 'overview_image')}
        for row in removed:
            for field in ('image', 'overview_image'):
                if row[field] not in retained_images:
                    (directory / row[field]).unlink(missing_ok=True)
        episodes += 1
        frames += len(removed)
        print(f'{directory.name}: removed {len(removed)} frames; {len(kept)} remain', flush=True)
    print(f'Trimmed {episodes} demos; removed {frames} idle frames.', flush=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Trim idle handover pauses in existing demos in place.')
    parser.add_argument('--data', required=True)
    trim_dataset(parser.parse_args().data)
