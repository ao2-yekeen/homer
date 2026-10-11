#!/usr/bin/env python3
"""Record a hand-guided recovery from the current pose, without replay or return."""
import argparse
import gzip
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

import cv2
import yaml

from record import ROOT, open_arm, read_positions, set_torque


class CorrectionEpisode:
    """Same JPEG/gzip training format; no autonomous samples or handover trim."""

    def __init__(self, directory, cfg, ticks, stamp, source_run):
        self.directory = directory
        directory.mkdir(parents=True)
        self.stream = gzip.open(directory / 'samples.jsonl.gz', 'wt')
        self.times = []
        self.count = 0
        self.metadata = dict(config=cfg, clock='Pi monotonic_ns',
                             start_state_ticks=ticks, start_state_timestamp_ns=stamp,
                             recording_start_ticks=ticks, return_completed_ns=None,
                             image_encoding='JPEG RGB; OpenCV input BGR',
                             capture_timestamp='Host read interval, not sensor exposure time',
                             joint_only=False, approach_end_frame=None,
                             assistance=dict(mode='manual-correction', source_run=source_run,
                                             action_source='hand-guided measured joints',
                                             torque_disabled=True))

    def append(self, wrist, overview, ticks, read_start, read_end):
        row = dict(state_ticks=ticks, read_start_ns=read_start, read_end_ns=read_end,
                   control=dict(phase='hand', action_source='human'))
        for frames, field, suffix, prefix in (
                (wrist, 'image', '', 'capture'),
                (overview, 'overview_image', '-overview', 'overview_capture')):
            frame, start, end = frames
            name = f'{self.count:06d}{suffix}.jpg'
            if not cv2.imwrite(str(self.directory / name), frame,
                               [cv2.IMWRITE_JPEG_QUALITY, 75]):
                raise RuntimeError(f'Image write failed: {name}')
            row.update({field: name, f'{prefix}_start_ns': start, f'{prefix}_end_ns': end})
        self.stream.write(json.dumps(row)+'\n')
        self.stream.flush()
        self.times.append(wrist[2])
        self.count += 1

    def finish(self, outcome, reason=None):
        self.stream.close()
        rate = ((self.count-1)*1e9/(self.times[-1]-self.times[0]) if self.count > 1 else None)
        self.metadata.update(outcome=outcome, reason=reason, samples=self.count, actual_fps=rate)
        (self.directory / 'metadata.json').write_text(json.dumps(self.metadata, indent=2)+'\n')
        print(f'{outcome}: {self.directory} ({self.count} samples, {rate} FPS)', flush=True)


def record_loop(cfg, packet, torque_register, cameras, poll_key, output, source_run):
    episode = None
    deadline = time.monotonic()
    print('CURRENT POSE: support the arm, then h releases torque and starts recording. '
          'f = successful placement/release; d = discard; q = quit.', flush=True)
    try:
        while True:
            key = poll_key()
            if key == ord('q'):
                break
            if key == ord('h') and episode is None:
                set_torque(packet, torque_register, cfg['servo_ids'], False)
                ticks, _, stamp = read_positions(packet, cfg)
                episode = CorrectionEpisode(output / f'episode-{time.time_ns()}', cfg,
                                            ticks, stamp, source_run)
                print('RECORDING CORRECTION: torque OFF. Guide into the container, '
                      'release and withdraw; f saves success.', flush=True)
            elif key in (ord('f'), ord('d')) and episode is not None:
                saved = episode
                episode = None
                saved.finish('success' if key == ord('f') and saved.count > 1 else 'discard')
                print('Torque remains OFF. q exits; h starts another correction '
                      'from the current pose.', flush=True)

            # Drain both streams while waiting as well, so recording starts
            # with current images rather than a queued pre-intervention scene.
            frames = []
            for name, camera in cameras:
                start = time.monotonic_ns()
                ok, frame = camera.read()
                end = time.monotonic_ns()
                if not ok:
                    raise RuntimeError(f'{name} camera read failed')
                frames.append((frame, start, end))
            if episode is not None:
                ticks, read_start, read_end = read_positions(packet, cfg)
                wrist, overview = frames
                episode.append(wrist, overview, ticks, read_start, read_end)
                if episode.count % max(1, round(cfg['fps'])) == 0:
                    print(f'RECORDING: {episode.count} samples | HAND GUIDANCE | f saves', flush=True)
            deadline += 1 / cfg['fps']
            time.sleep(max(0, deadline-time.monotonic()))
            deadline = max(deadline, time.monotonic())
    except BaseException as exc:
        if episode is not None:
            saved = episode
            episode = None
            saved.finish('invalid', str(exc) or type(exc).__name__)
        raise
    finally:
        if episode is not None:
            episode.finish('discard', 'Exited without marking success')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/rig.local.yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'data')
    parser.add_argument('--source-run', help='Policy run name for correction provenance')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Use an interactive terminal: ssh -t pi')
    cfg = yaml.safe_load(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    (ROOT / 'data').mkdir(exist_ok=True)
    cameras = []
    port = lock = terminal = None
    try:
        for name, settings in [('wrist', dict(device=cfg['camera'], width=cfg['width'],
                                             height=cfg['height'])),
                               ('overview', cfg['overview_camera'])]:
            cap = cv2.VideoCapture(settings['device'], cv2.CAP_V4L2)
            cameras.append((name, cap))
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings['height'])
            cap.set(cv2.CAP_PROP_FPS, cfg['fps'])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                raise RuntimeError(f'Cannot open {name} camera')
        port, packet, torque_register, lock = open_arm(cfg)
        terminal = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        def poll_key():
            return os.read(sys.stdin.fileno(), 1)[0] if select.select([sys.stdin], [], [], 0)[0] else -1

        record_loop(cfg, packet, torque_register, cameras, poll_key, args.output, args.source_run)
    except KeyboardInterrupt:
        print('\nStopped. Torque state unchanged; no automatic return.', flush=True)
    finally:
        if terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        if port is not None:
            port.closePort()
        if lock is not None:
            lock.close()
        for _, cap in cameras:
            cap.release()


if __name__ == '__main__':
    main()
