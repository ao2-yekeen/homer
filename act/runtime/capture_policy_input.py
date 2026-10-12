#!/usr/bin/env python3
"""Capture both ACT camera inputs and measured joints without motor commands."""
import argparse
import json
from pathlib import Path
import time

import cv2
import yaml

from record import checked, open_arm, read_positions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    cameras = []
    port = lock = None
    try:
        port, packet, _, lock = open_arm(cfg)
        frames = {}
        for name, device in [('wrist', cfg['camera']),
                             ('overview', cfg['overview_camera']['device'])]:
            camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
            cameras.append(camera)
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            camera.set(cv2.CAP_PROP_FPS, cfg['fps'])
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            for _ in range(4):
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError(f'Could not capture {name} camera')
            frames[name] = frame
        ticks, start, end = read_positions(packet, cfg)
        snapshot = dict(state_ticks=ticks, joint_names=cfg['joint_names'],
                        servo_limits=[[checked(packet.read2ByteTxRx(sid, 9))[0],
                                       checked(packet.read2ByteTxRx(sid, 11))[0]]
                                      for sid in cfg['servo_ids']],
                        read_start_ns=start, read_end_ns=end,
                        capture_time_ns=time.time_ns(), motor_commands_sent=False)
        for name, frame in frames.items():
            if not cv2.imwrite(str(args.output / f'{name}.jpg'), frame):
                raise RuntimeError(f'Could not save {name} image')
        (args.output / 'state.json').write_text(json.dumps(snapshot, indent=2)+'\n')
        print(json.dumps(snapshot), flush=True)
    finally:
        for camera in cameras:
            camera.release()
        if port is not None:
            port.closePort()
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    main()
