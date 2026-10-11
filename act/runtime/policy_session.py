#!/usr/bin/env python3
"""Pi motor/camera endpoint for a full ACT attempt; newline JSON over SSH."""
import argparse
import base64
import contextlib
import json
import select
from pathlib import Path
import sys
import time

import cv2
import yaml

from record import checked, open_arm, return_to_pose
from test_policy_chunk import clamp_ticks, write_targets
from connection_watch import watch_connection


def read_positions(packet, cfg):
    """Read feedback with the servo identity and SDK fault description."""
    start = time.monotonic_ns()
    values = []
    for sid, name in zip(cfg['servo_ids'], cfg['joint_names']):
        result = packet.ReadPosSpeed(sid)
        communication, status = result[-2:]
        if communication != 0:
            raise RuntimeError(f'Servo {sid} ({name}): communication error {communication}')
        if status:
            raise RuntimeError(f'Servo {sid} ({name}): '
                               f'{packet.getRxPacketError(status)} (status={status})')
        values.append(result[0])
    end = time.monotonic_ns()
    if end - start > 100_000_000:
        raise RuntimeError('Servo read exceeds one 10 Hz frame interval')
    if any(not 0 <= value <= 4095 for value in values):
        raise RuntimeError(f'Invalid raw servo tick reading: {values}')
    return values, start, end


def main(connection_finished=lambda: None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--steps', type=int, required=True)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    port = lock = None
    start = None
    cameras = {}
    report = dict(commands=[], requested_steps=args.steps, completed=False)

    def snapshot(index):
        images = {}
        for name, camera in cameras.items():
            for _ in range(4):
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError(f'Could not capture {name} camera')
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise RuntimeError(f'Could not encode {name} camera')
            raw = encoded.tobytes()
            (args.output / f'{index:03d}-{name}.jpg').write_bytes(raw)
            images[name] = base64.b64encode(raw).decode('ascii')
        ticks = read_positions(packet, cfg)[0]
        return dict(images=images, state_ticks=ticks, joint_names=cfg['joint_names'],
                    executed_steps=len(report['commands']))

    try:
        port, packet, torque_register, lock = open_arm(cfg)
        limits = [[checked(packet.read2ByteTxRx(sid, 9))[0],
                   checked(packet.read2ByteTxRx(sid, 11))[0]] for sid in cfg['servo_ids']]
        report['servo_limits'] = limits
        for name, device in [('wrist', cfg['camera']),
                             ('overview', cfg['overview_camera']['device'])]:
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            cameras[name] = cap
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, cfg['fps'])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        start = json.loads((Path(__file__).resolve().parents[1] /
                           'data/session-start-pose.json').read_text())['state_ticks']
        with contextlib.redirect_stdout(sys.stderr):
            return_to_pose(packet, torque_register, cfg, start, lambda: -1, keep_powered=True)
        report['initial_ticks'] = read_positions(packet, cfg)[0]
        chunk = 0
        while len(report['commands']) < args.steps:
            print(json.dumps(snapshot(chunk)), flush=True)
            # EOF/disconnection exits the loop and holds the measured pose below.
            line = sys.stdin.readline()
            if not line:
                break
            prediction = json.loads(line)
            if prediction.get('return_to_start'):
                raise KeyboardInterrupt
            if prediction.get('stop'):
                break
            if prediction['joint_names'] != cfg['joint_names']:
                raise ValueError('Policy joint order differs from the arm')
            raw_actions = prediction['actions'][:args.steps-len(report['commands'])]
            targets = [clamp_ticks(action, limits) for action in raw_actions]
            if not targets:
                raise ValueError('Empty action chunk')
            deadline = time.monotonic()
            for raw, target in zip(raw_actions, targets):
                if select.select([sys.stdin], [], [], 0)[0]:
                    control = sys.stdin.readline()
                    if control and json.loads(control).get('return_to_start'):
                        raise KeyboardInterrupt
                write_targets(packet, cfg['servo_ids'], target)
                deadline += 1 / cfg['fps']
                time.sleep(max(0, deadline-time.monotonic()))
                actual = read_positions(packet, cfg)[0]
                report['commands'].append(dict(prediction=raw, sent_ticks=target,
                                               measured_ticks=actual))
            chunk += 1
            (args.output / 'movement-report.json').write_text(json.dumps(report, indent=2)+'\n')
        report['completed'] = len(report['commands']) == args.steps
        # Hold before taking the final scene images, rather than advancing to
        # any pending goal while waiting for a picture or a network response.
        write_targets(packet, cfg['servo_ids'], clamp_ticks(read_positions(packet, cfg)[0], limits))
        report['hold_command_sent'] = True
        final = snapshot(chunk)
        final['finished'] = True
        # The PC closes its input on this message. Stop watching first so
        # normal completion cannot interrupt report/camera/serial cleanup.
        connection_finished()
        print(json.dumps(final), flush=True)
    except KeyboardInterrupt:
        report.update(completed=False, interrupted=True)
        if port is not None and start is not None:
            print('Interrupted: returning to saved start pose.', file=sys.stderr, flush=True)
            try:
                with contextlib.redirect_stdout(sys.stderr):
                    return_to_pose(packet, torque_register, cfg, start, lambda: -1,
                                   keep_powered=True)
                report['returned_to_start'] = True
                print('Start pose commands sent; arm holding.', file=sys.stderr, flush=True)
            except BaseException as exc:
                report['error'] = str(exc)
                print(f'Return to start failed: {exc}', file=sys.stderr, flush=True)
                raise
        connection_finished()
        raise SystemExit(130)
    except BaseException as exc:
        report.update(completed=False, error=str(exc))
        print(f'ACT stopped: {exc}. No further motor commands sent; '
              'torque state unchanged by cleanup.', file=sys.stderr, flush=True)
        raise
    finally:
        # A faulted servo may reject both feedback and commands. Do not mask
        # the original fault or re-enable torque here. Normal completion has
        # already sent the measured-pose hold above.
        try:
            (args.output / 'movement-report.json').write_text(json.dumps(report, indent=2)+'\n')
        finally:
            for camera in cameras.values():
                camera.release()
            if port is not None:
                port.closePort()
            if lock is not None:
                lock.close()


if __name__ == '__main__':
    with watch_connection() as connection_finished:
        main(connection_finished)
