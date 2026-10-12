#!/usr/bin/env python3
"""Record hand-guided or joystick demos, with optional approach replay."""
import argparse
import fcntl
import gzip
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

from trim_handover import trim_handover

ROOT = Path(__file__).resolve().parents[1]
RETURN_MAX_STEP_TICKS = 50
RETURN_STEP_INTERVAL_S = 0.05
RETURN_MOTOR_SPEED = 1000
RETURN_MOTOR_ACCELERATION = 50


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
    for service in ('soarm-bridge', 'robot-teleop', 'gamepad-teleop', 'gamepad-arm-test'):
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
        self.stream = gzip.open(directory / 'samples.jsonl.gz', 'wt')
        self.cfg = cfg
        self.start_state_ticks = start_state_ticks
        self.start_state_ns = start_state_ns
        self.recording_start_ticks = recording_start_ticks
        self.return_completed_ns = return_completed_ns
        self.times = []
        self.count = 0
        self.approach_end_frame = None
        self.assistance = None
        self.joint_only = False

    def append(self, frame, overview_frame, ticks, capture_start, capture_end,
               overview_start, overview_end, read_start, read_end, control=None):
        name = f'{self.count:06d}.jpg'
        # OpenCV encodes BGR as ordinary RGB JPEG; consumers decode to RGB once.
        if not cv2.imwrite(str(self.directory / name), frame,
                           [cv2.IMWRITE_JPEG_QUALITY, 75]):
            raise RuntimeError('Image write failed')
        row = dict(image=name, state_ticks=ticks, capture_start_ns=capture_start,
                   capture_end_ns=capture_end, read_start_ns=read_start,
                   read_end_ns=read_end)
        overview_name = f'{self.count:06d}-overview.jpg'
        if not cv2.imwrite(str(self.directory / overview_name), overview_frame,
                           [cv2.IMWRITE_JPEG_QUALITY, 75]):
            raise RuntimeError('Overview image write failed')
        row.update(overview_image=overview_name,
                   overview_capture_start_ns=overview_start,
                   overview_capture_end_ns=overview_end)
        if control is not None:
            row['control'] = control
        self.stream.write(json.dumps(row) + '\n')
        self.stream.flush()
        self.times.append(capture_end)
        self.count += 1

    def append_joint_state(self, ticks, read_start, read_end):
        """Store a replay-source sample without camera images or camera timing."""
        row = dict(state_ticks=ticks, read_start_ns=read_start,
                   read_end_ns=read_end)
        self.stream.write(json.dumps(row) + '\n')
        self.stream.flush()
        self.times.append(read_end)
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
                        return_completed_ns=self.return_completed_ns)
        if not self.joint_only:
            metadata.update(image_encoding='JPEG RGB; OpenCV input BGR',
                            capture_timestamp='Host read interval, not sensor exposure time')
        metadata.update(approach_end_frame=self.approach_end_frame,
                        assistance=self.assistance,
                        joint_only=self.joint_only)
        if (outcome == 'success' and not self.joint_only
                and (self.assistance or {}).get('mode') == 'replay-then-hand'):
            path = self.directory / 'samples.jsonl.gz'
            with gzip.open(path, 'rt') as stream:
                rows = [json.loads(line) for line in stream]
            rows, metadata, removed = trim_handover(rows, metadata)
            if removed:
                temporary = self.directory / 'samples.trimmed.jsonl.gz'
                with gzip.open(temporary, 'wt') as stream:
                    for row in rows:
                        stream.write(json.dumps(row)+'\n')
                temporary.replace(path)
                retained_images = {row[field] for row in rows
                                   for field in ('image', 'overview_image')}
                for row in removed:
                    for field in ('image', 'overview_image'):
                        if row[field] not in retained_images:
                            (self.directory / row[field]).unlink(missing_ok=True)
                self.count = len(rows)
                self.approach_end_frame = metadata['approach_end_frame']
                print(f'Removed {len(removed)} idle handover frames.', flush=True)
        (self.directory / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        print(f'{outcome}: {self.directory} ({self.count} samples, {rate} FPS)', flush=True)


def review(directory):
    import numpy as np
    metadata = json.loads((directory / 'metadata.json').read_text())
    path = directory / 'samples.jsonl.gz'
    stream = gzip.open(path, 'rt') if path.exists() else (directory / 'samples.jsonl').open()
    with stream:
        rows = [json.loads(line) for line in stream]
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
        for index, row in enumerate(rows):
            frame = cv2.imread(str(directory / row['image']))
            if frame is None:
                raise RuntimeError('Missing episode image')
            overview = cv2.imread(str(directory / row['overview_image']))
            if overview is None:
                raise RuntimeError('Missing episode overview image')
            overview = cv2.resize(overview, (cfg['width'], cfg['height']))
            frame = cv2.hconcat([frame, overview])
            cv2.putText(frame, f"frame {index}: {row['state_ticks']}", (10, 25),
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
    ap.add_argument('--joystick', action='store_true', help='Record powered joystick demonstrations')
    ap.add_argument('--approach-source', action='store_true', help='Record a reusable approach and save it when p is pressed')
    ap.add_argument('--joystick-check', action='store_true', help='Print joystick state for 10 seconds, no arm/camera access')
    ap.add_argument('--replay', type=Path, help='Successful episode whose approach is replayed before hand guidance')
    ap.add_argument('--until-frame', type=int, help='Last source frame to replay; default: p marker in source metadata')
    ap.add_argument('--replay-speed', type=float, default=1., help='Source timing multiplier, 0 < speed <= 1 (default 1)')
    args = ap.parse_args()
    if args.review:
        review(args.review)
        return
    cfg = yaml.safe_load(args.config.read_text())
    if not 0 < args.replay_speed <= 1:
        ap.error('--replay-speed must be > 0 and <= 1')
    if args.until_frame is not None and not args.replay:
        ap.error('--until-frame requires --replay')
    if args.replay:
        args.joystick = False  # Replay ends with torque off for hand guidance.
    if args.joystick_check:
        from assisted import XboxJoystick
        joystick = XboxJoystick(cfg['joystick']['device'])
        try:
            print(f'Input-only check: {joystick.name}. Move sticks/D-pad and press R1/B.', flush=True)
            for _ in range(50):
                print(joystick.snapshot(), flush=True)
                time.sleep(.2)
        finally:
            joystick.close()
        return
    if 'overview_camera' not in cfg and not args.approach_source:
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
    assistance = None
    joystick = None
    terminal_settings = None
    try:
        if not args.check and not args.approach_source:
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
            set_torque(packet, torque_register, cfg['servo_ids'], False)
            print('Startup safety: torque disabled on all six joints.', flush=True)
        session_start_pose, session_start_pose_ns = get_session_start_pose(
            ROOT / 'data' / 'session-start-pose.json', cfg, positions,
            position_read_end)
        if args.joystick or args.replay:
            from assisted import XboxJoystick, AssistedControl, read_limits, load_prefix
            limits = read_limits(packet, cfg, checked)
            prefix = load_prefix(args.replay, args.until_frame, cfg, limits,
                                 session_start_pose) if args.replay else None
            joystick = XboxJoystick(cfg['joystick']['device']) if args.joystick else None
            assistance = AssistedControl(packet, torque_register, cfg, joystick,
                                         limits, checked, set_torque, prefix, args.replay_speed)
        print(f'Session start pose loaded: {session_start_pose}', flush=True)
        if assistance and assistance.prefix:
            print('READY — press s to return to the INITIAL POSE; recording starts there and the source replays automatically.\n'
                  'After replay the arm holds position. Support it, then press h to release torque and finish by hand.', flush=True)
        else:
            print('READY — press s to return to the saved pose and start a demo.\n'
                  's=start, f=finish successful episode, d=discard, q=quit; q or x stops a return.', flush=True)
        if joystick:
            print('JOYSTICK MODE: torque is enabled only while returning/moving; it is released on exit.\n'
                  'Hold R1 + sticks/D-pad to move; B pauses; p marks the approach endpoint.\n'
                  'x aborts and exits; support the arm before x/q releases torque.', flush=True)
        if args.approach_source:
            print('APPROACH SOURCE MODE: guide the arm by hand with torque off; p saves through this frame and exits.', flush=True)
        if args.no_preview:
            if not sys.stdin.isatty():
                raise RuntimeError('Use an interactive terminal: ssh -t pi')
            terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        period = 1 / cfg['fps']
        deadline = time.monotonic()
        while True:
            if assistance and episode:
                ticks, _, _ = read_positions(packet, cfg)
                previous_phase = assistance.phase
                assistance.update(ticks, time.monotonic())
                if previous_phase not in ('joystick', 'holding', 'hand') and assistance.phase in ('joystick', 'holding', 'hand'):
                    episode.approach_end_frame = episode.count
            if not args.approach_source:
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
                if args.approach_source:
                    episode.append_joint_state(ticks, read_start, read_end)
                else:
                    episode.append(frame, overview_frame, ticks, capture_start,
                                   capture_end, overview_start, overview_end,
                                   read_start, read_end,
                                   dict(phase=assistance.phase, target_ticks=assistance.target,
                                        axes=assistance.last_axes) if assistance else None)
                if args.no_preview and episode.count % max(1, round(cfg['fps'])) == 0:
                    if assistance:
                        if assistance.phase == 'holding':
                            control_status = 'HOLDING; torque ON; support arm then press h'
                        elif assistance.phase == 'hand':
                            control_status = 'HAND GUIDANCE; torque off; f saves'
                        elif assistance.phase not in ('joystick', 'hand'):
                            control_status = assistance.phase.upper()
                        elif not assistance.ready:
                            control_status = 'WAITING: centre ALL sticks/D-pad and release R1'
                        elif assistance.was_held:
                            control_status = f'R1 HELD; axes={[round(v, 2) for v in assistance.last_axes]}'
                        else:
                            control_status = 'READY: hold R1 (right bumper) + move a stick'
                    else:
                        control_status = 'HAND GUIDANCE; torque off; f saves'
                    print(f'RECORDING: {episode.count} samples saved | {control_status}', flush=True)
            if args.no_preview:
                key = os.read(sys.stdin.fileno(), 1)[0] if select.select([sys.stdin], [], [], 0)[0] else -1
            else:
                status = ('HOLDING: support arm, h releases torque'
                          if episode and assistance and assistance.phase == 'holding'
                          else 'RECORDING' if episode else 's:start f:finish d:discard q:quit')
                cv2.putText(frame, status,
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
                cv2.putText(overview_frame, 'LOGITECH OVERVIEW', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
                cv2.imshow('ACT wrist demonstration', frame)
                cv2.imshow('ACT Logitech overview', overview_frame)
                key = cv2.waitKey(1) & 255
            if key == ord('s') and episode is None:
                print('Returning to saved start pose.', flush=True)
                def poll_key():
                    if joystick:
                        _, _, stopped = joystick.snapshot()
                        if stopped:
                            return ord('x')
                    if args.no_preview:
                        return os.read(sys.stdin.fileno(), 1)[0] if select.select([sys.stdin], [], [], 0)[0] else -1
                    if args.approach_source:
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
                if assistance:
                    assistance.enabled = True
                cancel_key = return_to_pose(packet, torque_register, cfg,
                                            session_start_pose, poll_key,
                                            keep_powered=assistance is not None)
                if cancel_key != -1:
                    print('Return canceled; torque is off. Press s to retry or q to quit.', flush=True)
                    if cancel_key == ord('q'):
                        break
                    if assistance and cancel_key == ord('x'):
                        break
                    continue
                ticks, read_start, read_end = read_positions(packet, cfg)
                return_completed_ns = time.monotonic_ns()
                if assistance:
                    assistance.begin(ticks, time.monotonic())
                episode = Episode(ROOT / 'data' / f'episode-{time.time_ns()}',
                                  cfg, session_start_pose, session_start_pose_ns,
                                  ticks, return_completed_ns)
                episode.joint_only = args.approach_source
                if assistance:
                    episode.assistance = dict(mode='replay-then-hand' if assistance.prefix else 'joystick',
                                              replay_source=assistance.prefix['source'] if assistance.prefix else None,
                                              replay_until_frame=assistance.prefix['until_frame'] if assistance.prefix else None,
                                              replay_speed=args.replay_speed)
                if args.approach_source:
                    episode.append_joint_state(ticks, read_start, read_end)
                else:
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
                    episode.append(frame, overview_frame, ticks, capture_start,
                                   capture_end, overview_start, overview_end,
                                   read_start, read_end,
                                   dict(phase=assistance.phase, target_ticks=assistance.target,
                                        axes=assistance.last_axes) if assistance else None)
                print(f'RECORDING STARTED at initial pose: {episode.directory}', flush=True)
            elif key == ord('s'):
                print('Already recording. Press f to save or d to discard.', flush=True)
            elif key == ord('h') and episode and assistance and assistance.phase == 'holding':
                assistance.release_for_hand()
            elif key in (ord('f'), ord('d')) and episode:
                if key == ord('f') and assistance and assistance.phase not in ('joystick', 'hand'):
                    print('Finish requires replay completion; discard an interrupted replay with d.', flush=True)
                    continue
                if assistance:
                    assistance.finish(ticks)
                episode.finish('success' if key == ord('f') and episode.count > 1 else 'discard')
                episode = None
                print('DEMO SAVED — press s to return to the initial pose and record/replay the next demo.', flush=True)
            elif key in (ord('f'), ord('d')):
                print('No episode recording. Press s to start.', flush=True)
            elif key == ord('p') and episode:
                if assistance and assistance.phase not in ('joystick', 'hand'):
                    print('Mark the approach only after replay completes.', flush=True)
                else:
                    episode.approach_end_frame = episode.count - 1
                    print(f'APPROACH END MARKED: frame {episode.approach_end_frame}', flush=True)
                    if args.approach_source:
                        if assistance:
                            assistance.finish(ticks)
                        episode.assistance = dict(mode='approach-source')
                        episode.finish('success')
                        episode = None
                        break
            elif key == ord('q') or (key == ord('x') and assistance):
                break
            deadline += period
            time.sleep(max(0, deadline - time.monotonic()))
            deadline = max(deadline, time.monotonic())
    except BaseException as exc:
        if episode:
            episode.finish('invalid', str(exc))
            episode = None
        if (isinstance(exc, KeyboardInterrupt) and port
                and session_start_pose is not None and not args.approach_source):
            print('\nCtrl+C — returning to the saved start pose before exiting. '
                  'Press Ctrl+C again to stop the return.', flush=True)
            return_to_pose(packet, torque_register, cfg, session_start_pose,
                           lambda: -1)
        else:
            raise
    finally:
        cleanup_error = None
        try:
            if assistance:
                assistance.close()
            elif joystick:
                joystick.close()
        except Exception as exc:
            cleanup_error = exc
        finally:
            if port:
                port.closePort()
                port = None
        if terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        if episode:
            episode.finish('discard', 'Exited before marking success')
        if lock:
            lock.close()
        if cap:
            cap.release()
        if overview_cap:
            overview_cap.release()
        cv2.destroyAllWindows()
        if cleanup_error:
            raise cleanup_error


if __name__ == '__main__':
    main()
