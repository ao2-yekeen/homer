#!/usr/bin/env python3
"""USB wrist-camera / six-servo demonstration recorder. No automatic motion."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import select
import termios
import tty
import time

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[1]


def checked(result):
    if result[-2:] != (0, 0):
        raise RuntimeError(f'Servo communication failed: {result[-2:]}')
    return result


def read_positions(packet, cfg):
    start = time.monotonic_ns()
    values = [checked(packet.ReadPosSpeed(i))[0] for i in cfg['servo_ids']]
    end = time.monotonic_ns()
    if end - start > 100_000_000:
        raise RuntimeError('Servo read exceeds one 10 Hz frame interval')
    if any(not 0 <= value <= 4095 for value in values):
        raise RuntimeError(f'Invalid raw servo tick reading: {values}')
    return values, start, end


def open_arm(cfg):
    # Linux serial exclusivity plus a cooperative lock; no fallback to an unknown USB port.
    for service in ('soarm-bridge', 'robot-teleop'):
        if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', service],
                          check=False).returncode == 0:
            raise RuntimeError(f'Stop {service}.service before accessing the arm')
    owner = subprocess.run(['fuser', cfg['serial']], capture_output=True, check=False)
    if owner.returncode != 1:
        raise RuntimeError('Serial device is in use, missing, or ownership check failed: ' + owner.stderr.decode())
    lock = open(ROOT / 'data' / 'serial.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sys.path.insert(0, cfg['sdk_path'])
    from STservo_sdk import PortHandler, sts, STS_TORQUE_ENABLE
    port = PortHandler(cfg['serial'])
    try:
        if not port.openPort() or not port.setBaudRate(cfg['baud']):
            raise RuntimeError('Cannot open configured arm serial device')
        import termios
        fcntl.ioctl(port.ser.fileno(), termios.TIOCEXCL)
        packet = sts(port)
        read_positions(packet, cfg)
        return port, packet, STS_TORQUE_ENABLE, lock
    except BaseException:
        port.closePort()
        lock.close()
        raise


class Episode:
    def __init__(self, directory, cfg):
        self.directory = directory
        directory.mkdir(parents=True)
        self.stream = (directory / 'samples.jsonl').open('w')
        self.cfg = cfg
        self.times = []
        self.count = 0

    def append(self, frame, ticks, capture_start, capture_end, read_start, read_end):
        name = f'{self.count:06d}.jpg'
        # OpenCV encodes BGR as ordinary RGB JPEG; consumers decode to RGB once.
        if not cv2.imwrite(str(self.directory / name), frame):
            raise RuntimeError('Image write failed')
        row = dict(image=name, state_ticks=ticks, capture_start_ns=capture_start,
                   capture_end_ns=capture_end, read_start_ns=read_start, read_end_ns=read_end)
        self.stream.write(json.dumps(row) + '\n')
        self.stream.flush()
        self.times.append(capture_end)
        self.count += 1

    def finish(self, outcome, reason=None):
        self.stream.close()
        rate = ((self.count - 1) * 1e9 / (self.times[-1] - self.times[0])
                if self.count > 1 else None)
        metadata = dict(outcome=outcome, reason=reason, samples=self.count,
                        actual_fps=rate, clock='Pi monotonic_ns', config=self.cfg,
                        image_encoding='JPEG RGB; OpenCV input BGR',
                        capture_timestamp='Host read interval, not sensor exposure time')
        (self.directory / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        print(f'{outcome}: {self.directory} ({self.count} samples, {rate} FPS)', flush=True)


def review(directory):
    import numpy as np
    metadata = json.loads((directory / 'metadata.json').read_text())
    rows = [json.loads(line) for line in (directory / 'samples.jsonl').read_text().splitlines()]
    if len(rows) < 2:
        raise RuntimeError('Need at least two samples for review')
    cfg = metadata['config']
    writer = cv2.VideoWriter(str(directory / 'review.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                             metadata['actual_fps'], (cfg['width'], cfg['height']))
    if not writer.isOpened():
        raise RuntimeError('Cannot create review video')
    try:
        for row in rows:
            frame = cv2.imread(str(directory / row['image']))
            if frame is None:
                raise RuntimeError('Missing episode image')
            cv2.putText(frame, str(row['state_ticks']), (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1)
            writer.write(frame)
    finally:
        writer.release()
    plot = np.full((720, 1000, 3), 255, dtype=np.uint8)
    times = np.array([r['capture_end_ns'] for r in rows], dtype=np.float64)
    x = (80 + (times - times[0]) / (times[-1] - times[0]) * 880).astype(int)
    for j, name in enumerate(cfg['joint_names']):
        top = j * 120
        low, high = 0, 4095
        values = np.array([r['state_ticks'][j] for r in rows])
        y = (top + 105 - (values - low) / (high - low) * 80).astype(int)
        cv2.putText(plot, f'{name}: {low}..{high} ticks', (10, top + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 1)
        cv2.polylines(plot, [np.column_stack((x, y)).astype(np.int32)], False, (180, 70, 0), 2)
    if not cv2.imwrite(str(directory / 'joints.png'), plot):
        raise RuntimeError('Cannot save joint plot')
    print(f"Review saved: {directory / 'review.avi'} and {directory / 'joints.png'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, default=ROOT / 'config/rig.local.yaml')
    ap.add_argument('--no-preview', action='store_true', help='Use terminal keys, suitable for SSH')
    ap.add_argument('--camera-check', action='store_true', help='Save camera sample, no servo access')
    ap.add_argument('--review', type=Path, help='Export episode review video and joint plot without hardware')
    ap.add_argument('--check', action='store_true', help='Read positions/torque only, no servo writes')
    args = ap.parse_args()
    if args.review:
        review(args.review)
        return
    cfg = yaml.safe_load(args.config.read_text())
    if cfg['servo_ids'] != list(range(1, 7)) or len(cfg['joint_names']) != 6:
        raise RuntimeError('Expected six ordered servos 1..6')
    if not 0 < cfg['fps'] <= 30:
        raise RuntimeError('FPS must be >0 and <=30')
    (ROOT / 'data').mkdir(exist_ok=True)
    cap = None
    port = None
    lock = None
    episode = None
    terminal_settings = None
    try:
        if not args.check:
            cap = cv2.VideoCapture(cfg['camera'], cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['height'])
            cap.set(cv2.CAP_PROP_FPS, cfg['fps'])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                raise RuntimeError('Cannot open wrist camera')
            for _ in range(5):
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('Wrist camera read failed')
            if frame.shape[:2] != (cfg['height'], cfg['width']):
                raise RuntimeError(f'Unexpected camera resolution: {frame.shape}')
            if args.camera_check:
                started = time.monotonic()
                for _ in range(20):
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('Camera check failed')
                measured = 20 / (time.monotonic() - started)
                path = ROOT / 'data/camera-check.jpg'
                if not cv2.imwrite(str(path), frame):
                    raise RuntimeError('Camera image save failed')
                print(f'Saved {path}; reported camera FPS={cap.get(cv2.CAP_PROP_FPS)}; measured={measured:.2f} FPS')
                return
        port, packet, torque_register, lock = open_arm(cfg)
        positions, _, _ = read_positions(packet, cfg)
        torque = [checked(packet.read1ByteTxRx(i, torque_register))[0] for i in cfg['servo_ids']]
        print(f'Positions: {positions}; torque: {torque}', flush=True)
        if args.check:
            return
        if any(torque):
            raise RuntimeError('All six joints must be in verified manual torque-off guidance mode')
        print('READY — press s to start. Keys act immediately; no Enter needed and key echo is off.\n'
              's=start, f=finish successful episode, d=discard, q=quit. No torque restoration on exit.', flush=True)
        if args.no_preview:
            if not sys.stdin.isatty():
                raise RuntimeError('Use an interactive terminal: ssh -t pi')
            terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        period = 1 / cfg['fps']
        deadline = time.monotonic()
        while True:
            capture_start = time.monotonic_ns()
            ok, frame = cap.read()
            capture_end = time.monotonic_ns()
            if not ok:
                raise RuntimeError('Camera disconnected/read failed')
            if capture_end - capture_start > 200_000_000:
                raise RuntimeError('Camera read took over 200 ms')
            ticks, read_start, read_end = read_positions(packet, cfg)
            if episode:
                episode.append(frame, ticks, capture_start, capture_end, read_start, read_end)
                if args.no_preview and episode.count % max(1, round(cfg['fps'])) == 0:
                    print(f'RECORDING: {episode.count} samples saved', flush=True)
            cv2.putText(frame, 'RECORDING' if episode else 's:start f:finish d:discard q:quit',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
            if args.no_preview:
                key = os.read(sys.stdin.fileno(), 1)[0] if select.select([sys.stdin], [], [], 0)[0] else -1
            else:
                cv2.imshow('ACT wrist demonstration', frame)
                key = cv2.waitKey(1) & 255
            if key == ord('s') and episode is None:
                episode = Episode(ROOT / 'data' / f'episode-{time.time_ns()}', cfg)
                print(f'RECORDING STARTED: {episode.directory}', flush=True)
            elif key == ord('s'):
                print('Already recording. Press f to save or d to discard.', flush=True)
            elif key in (ord('f'), ord('d')) and episode:
                episode.finish('success' if key == ord('f') and episode.count > 1 else 'discard')
                episode = None
                print('READY — press s for the next episode.', flush=True)
            elif key in (ord('f'), ord('d')):
                print('No episode recording. Press s to start.', flush=True)
            elif key == ord('q'):
                break
            deadline += period
            time.sleep(max(0, deadline - time.monotonic()))
            deadline = max(deadline, time.monotonic())
    except BaseException as exc:
        if episode:
            episode.finish('invalid', str(exc))
            episode = None
        raise
    finally:
        if terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        if episode:
            episode.finish('discard', 'Exited before marking success')
        if port:
            port.closePort()
        if lock:
            lock.close()
        if cap:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
