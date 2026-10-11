#!/usr/bin/env python3
"""Record hand-guided demos after returning the arm to its session start pose."""
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
RETURN_MAX_STEP_TICKS = 50
RETURN_STEP_INTERVAL_S = 0.05
RETURN_MOTOR_SPEED = 1000
RETURN_MOTOR_ACCELERATION = 50
START_RECORD_TOLERANCE_TICKS = 30


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


def get_session_start_pose(path, cfg, live_ticks, live_timestamp_ns):
    if path.exists():
        saved = json.loads(path.read_text())
        if saved.get('joint_names') != cfg['joint_names']:
            raise RuntimeError('Saved start pose joint order does not match this rig')
        ticks = saved.get('state_ticks')
        if (not isinstance(ticks, list) or len(ticks) != len(cfg['servo_ids'])
                or any(not isinstance(v, int) or not 0 <= v <= 4095 for v in ticks)):
            raise RuntimeError('Saved start pose has invalid servo ticks')
        return ticks, None
    saved = dict(state_ticks=live_ticks, joint_names=cfg['joint_names'])
    path.write_text(json.dumps(saved, indent=2) + '\n')
    return live_ticks, live_timestamp_ns


def set_torque(packet, torque_register, servo_ids, enabled):
    value = 1 if enabled else 0
    errors = []
    for servo_id in servo_ids:
        try:
            checked(packet.write1ByteTxRx(servo_id, torque_register, value))
        except Exception as exc:
            errors.append(f'{servo_id}: {exc}')
    if errors:
        mode = 'enable' if enabled else 'disable'
        raise RuntimeError(f'Could not {mode} torque on servos: ' + '; '.join(errors))
    expected = 1 if enabled else 0
    for servo_id in servo_ids:
        actual = checked(packet.read1ByteTxRx(servo_id, torque_register))[0]
        if actual != expected:
            raise RuntimeError(
                f'Torque state verification failed for servo {servo_id}: '
                f'expected {expected}, got {actual}')


def return_to_pose(packet, torque_register, cfg, target_ticks, poll_key, keep_powered=False):
    """Return to start; keep a successful joystick return powered, release on errors."""
    servo_ids = cfg['servo_ids']
    current, _, _ = read_positions(packet, cfg)
    if len(target_ticks) != len(servo_ids) or any(not 0 <= v <= 4095 for v in target_ticks):
        raise RuntimeError('Saved start pose is invalid')

    # Manual guidance can place a joint outside its programmed position limits.
    # Clamp the initial hold command, but do not refuse the return over limits.
    return_start = []
    for servo_id, actual in zip(servo_ids, current):
        minimum = checked(packet.read2ByteTxRx(servo_id, 9))[0]
        maximum = checked(packet.read2ByteTxRx(servo_id, 11))[0]
        # Start at the nearest legal goal, then move toward the saved pose.
        return_start.append(min(max(actual, minimum), maximum))

    torque_enabled = False
    completed = False
    cancel_key = -1
    try:
        # Replace any stale goals before enabling torque so the arm initially
        # holds where it is now rather than jumping to an older servo goal.
        for servo_id, position in zip(servo_ids, return_start):
            checked(packet.WritePosEx(servo_id, position,
                                      RETURN_MOTOR_SPEED, RETURN_MOTOR_ACCELERATION))
        torque_enabled = True
        set_torque(packet, torque_register, servo_ids, True)

        max_delta = max(abs(dst - src) for src, dst in zip(return_start, target_ticks))
        steps = max(1, (max_delta + RETURN_MAX_STEP_TICKS - 1) // RETURN_MAX_STEP_TICKS)
        last_report = time.monotonic()
        for step in range(1, steps + 1):
            key = poll_key()
            if key in (ord('q'), ord('x')):
                cancel_key = key
                break
            targets = [round(src + (dst - src) * step / steps)
                       for src, dst in zip(return_start, target_ticks)]
            for servo_id, target in zip(servo_ids, targets):
                checked(packet.WritePosEx(servo_id, target,
                                          RETURN_MOTOR_SPEED, RETURN_MOTOR_ACCELERATION))
            if time.monotonic() - last_report >= 1:
                print(f'RETURNING TO START: step {step}/{steps}', flush=True)
                last_report = time.monotonic()
            time.sleep(RETURN_STEP_INTERVAL_S)

        completed = cancel_key == -1
        if completed:
            print('START POSE COMMANDS SENT.', flush=True)
        return cancel_key
    finally:
        if torque_enabled and (not keep_powered or not completed):
            set_torque(packet, torque_register, servo_ids, False)


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
    def __init__(self, directory, cfg, start_state_ticks, start_state_ns,
                 recording_start_ticks, return_completed_ns):
        self.directory = directory
        directory.mkdir(parents=True)
        self.stream = (directory / 'samples.jsonl').open('w')
        self.cfg = cfg
        self.start_state_ticks = start_state_ticks
        self.start_state_ns = start_state_ns
        self.recording_start_ticks = recording_start_ticks
        self.return_completed_ns = return_completed_ns
        self.times = []
        self.count = 0

    def append(self, frame, overview_frame, ticks, capture_start, capture_end,
               overview_start, overview_end, read_start, read_end):
        name = f'{self.count:06d}.jpg'
        # OpenCV encodes BGR as ordinary RGB JPEG; consumers decode to RGB once.
        if not cv2.imwrite(str(self.directory / name), frame):
            raise RuntimeError('Image write failed')
        row = dict(image=name, state_ticks=ticks, capture_start_ns=capture_start,
                   capture_end_ns=capture_end, read_start_ns=read_start,
                   read_end_ns=read_end)
        overview_name = f'{self.count:06d}-overview.jpg'
        if not cv2.imwrite(str(self.directory / overview_name), overview_frame):
            raise RuntimeError('Overview image write failed')
        row.update(overview_image=overview_name,
                   overview_capture_start_ns=overview_start,
                   overview_capture_end_ns=overview_end)
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
                        start_state_ticks=self.start_state_ticks,
                        start_state_timestamp_ns=self.start_state_ns,
                        recording_start_ticks=self.recording_start_ticks,
                        return_completed_ns=self.return_completed_ns,
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
    overview_cfg = cfg['overview_camera']
    review_width = cfg['width'] * 2
    writer = cv2.VideoWriter(str(directory / 'review.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                             metadata['actual_fps'], (review_width, cfg['height']))
    if not writer.isOpened():
        raise RuntimeError('Cannot create review video')
    try:
        for row in rows:
            frame = cv2.imread(str(directory / row['image']))
            if frame is None:
                raise RuntimeError('Missing episode image')
            overview = cv2.imread(str(directory / row['overview_image']))
            if overview is None:
                raise RuntimeError('Missing episode overview image')
            overview = cv2.resize(overview, (cfg['width'], cfg['height']))
            frame = cv2.hconcat([frame, overview])
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
    if 'overview_camera' not in cfg:
        raise RuntimeError('Configuration must include the Logitech overview_camera')
    if cfg['servo_ids'] != list(range(1, 7)) or len(cfg['joint_names']) != 6:
        raise RuntimeError('Expected six ordered servos 1..6')
    if not 0 < cfg['fps'] <= 30:
        raise RuntimeError('FPS must be >0 and <=30')
    (ROOT / 'data').mkdir(exist_ok=True)
    session_start_pose = None
    session_start_pose_ns = None
    cap = None
    overview_cap = None
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
            overview_cfg = cfg['overview_camera']
            overview_cap = cv2.VideoCapture(overview_cfg['device'], cv2.CAP_V4L2)
            overview_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            overview_cap.set(cv2.CAP_PROP_FRAME_WIDTH, overview_cfg['width'])
            overview_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, overview_cfg['height'])
            overview_cap.set(cv2.CAP_PROP_FPS, cfg['fps'])
            overview_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not overview_cap.isOpened():
                raise RuntimeError('Cannot open configured Logitech overview camera')
            for _ in range(5):
                overview_ok, overview_frame = overview_cap.read()
                if not overview_ok:
                    raise RuntimeError('Logitech overview camera read failed')
            if overview_frame.shape[:2] != (overview_cfg['height'], overview_cfg['width']):
                raise RuntimeError(f'Unexpected overview camera resolution: {overview_frame.shape}')
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
                message = f'Saved {path}; reported camera FPS={cap.get(cv2.CAP_PROP_FPS)}; measured={measured:.2f} FPS'
                overview_path = ROOT / 'data/camera-check-overview.jpg'
                if not cv2.imwrite(str(overview_path), overview_frame):
                    raise RuntimeError('Overview camera image save failed')
                message += f'; saved {overview_path}'
                print(message)
                return
        port, packet, torque_register, lock = open_arm(cfg)
        positions, _, position_read_end = read_positions(packet, cfg)
        torque = [checked(packet.read1ByteTxRx(i, torque_register))[0] for i in cfg['servo_ids']]
        print(f'Positions: {positions}; torque: {torque}', flush=True)
        if args.check:
            return
        if any(torque):
            raise RuntimeError('All six joints must be in verified manual torque-off guidance mode')
        session_start_pose, session_start_pose_ns = get_session_start_pose(
            ROOT / 'data' / 'session-start-pose.json', cfg, positions,
            position_read_end)
        print(f'Session start pose loaded: {session_start_pose}', flush=True)
        print('READY — press s to return to the saved pose, release torque, and start a demo.\n'
              's=start, f=finish successful episode, d=discard, q=quit; q or x stops a return.', flush=True)
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
            overview_start = time.monotonic_ns()
            overview_ok, overview_frame = overview_cap.read()
            overview_end = time.monotonic_ns()
            if not overview_ok:
                raise RuntimeError('Logitech overview camera disconnected/read failed')
            if overview_end - overview_start > 200_000_000:
                raise RuntimeError('Logitech overview camera read took over 200 ms')
            ticks, read_start, read_end = read_positions(packet, cfg)
            if episode:
                episode.append(frame, overview_frame, ticks, capture_start,
                               capture_end, overview_start, overview_end,
                               read_start, read_end)
                if args.no_preview and episode.count % max(1, round(cfg['fps'])) == 0:
                    print(f'RECORDING: {episode.count} samples saved', flush=True)
            cv2.putText(frame, 'RECORDING' if episode else 's:start f:finish d:discard q:quit',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
            cv2.putText(overview_frame, 'LOGITECH OVERVIEW', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
            if args.no_preview:
                key = os.read(sys.stdin.fileno(), 1)[0] if select.select([sys.stdin], [], [], 0)[0] else -1
            else:
                cv2.imshow('ACT wrist demonstration', frame)
                cv2.imshow('ACT Logitech overview', overview_frame)
                key = cv2.waitKey(1) & 255
            if key == ord('s') and episode is None:
                print('Returning to saved start pose; torque will be released for hand guidance.', flush=True)
                def poll_key():
                    if args.no_preview:
                        return os.read(sys.stdin.fileno(), 1)[0] if select.select([sys.stdin], [], [], 0)[0] else -1
                    wrist_ok, wrist = cap.read()
                    overview_ok, overview = overview_cap.read()
                    if not wrist_ok or not overview_ok:
                        raise RuntimeError('Camera read failed during return to start pose')
                    cv2.putText(wrist, 'RETURNING TO START (q/x aborts)', (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                    cv2.putText(overview, 'RETURNING TO START (q/x aborts)', (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                    cv2.imshow('ACT wrist demonstration', wrist)
                    cv2.imshow('ACT Logitech overview', overview)
                    return cv2.waitKey(1) & 255
                cancel_key = return_to_pose(packet, torque_register, cfg,
                                            session_start_pose, poll_key)
                if cancel_key != -1:
                    print('Return canceled; torque is off. Press s to retry or q to quit.', flush=True)
                    if cancel_key == ord('q'):
                        break
                    continue
                capture_start = time.monotonic_ns()
                ok, frame = cap.read()
                capture_end = time.monotonic_ns()
                if not ok:
                    raise RuntimeError('Wrist camera read failed after return to start pose')
                overview_start = time.monotonic_ns()
                overview_ok, overview_frame = overview_cap.read()
                overview_end = time.monotonic_ns()
                if not overview_ok:
                    raise RuntimeError('Logitech camera read failed after return to start pose')
                ticks, read_start, read_end = read_positions(packet, cfg)
                if any(abs(actual - target) > START_RECORD_TOLERANCE_TICKS
                       for actual, target in zip(ticks, session_start_pose)):
                    raise RuntimeError(
                        'Arm moved away from the saved start pose after torque was released: '
                        f'target={session_start_pose}, actual={ticks}')
                return_completed_ns = time.monotonic_ns()
                episode = Episode(ROOT / 'data' / f'episode-{time.time_ns()}',
                                  cfg, session_start_pose, session_start_pose_ns,
                                  ticks, return_completed_ns)
                episode.append(frame, overview_frame, ticks, capture_start,
                               capture_end, overview_start, overview_end,
                               read_start, read_end)
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
        if overview_cap:
            overview_cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
