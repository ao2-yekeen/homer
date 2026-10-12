#!/usr/bin/env python3
"""Run one predicted chunk, clamp motor goals, then return to start and hold."""
import argparse
import json
import math
from pathlib import Path
import time

import yaml

from record import (checked, open_arm, read_positions, return_to_pose, set_torque,
                    RETURN_MOTOR_SPEED, RETURN_MOTOR_ACCELERATION)


def clamp_ticks(action, limits):
    if len(action) != len(limits) or not all(math.isfinite(value) for value in action):
        raise ValueError('Expected one finite target per motor')
    return [min(high, max(low, round(value)))
            for value, (low, high) in zip(action, limits)]


def write_targets(packet, servo_ids, targets):
    for sid, tick in zip(servo_ids, targets):
        checked(packet.WritePosEx(sid, tick, RETURN_MOTOR_SPEED, RETURN_MOTOR_ACCELERATION))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--actions', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--max-steps', type=int, default=50)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    prediction = json.loads(args.actions.read_text())
    if prediction['joint_names'] != cfg['joint_names']:
        raise ValueError('Policy joint order differs from the arm')
    start_pose = json.loads((Path(__file__).resolve().parents[1] /
                            'data/session-start-pose.json').read_text())['state_ticks']
    port = lock = None
    powered = False
    report = dict(commands=[], returned_to_start=False, torque_left_enabled=False)
    try:
        port, packet, torque_register, lock = open_arm(cfg)
        limits = [[checked(packet.read2ByteTxRx(sid, 9))[0],
                   checked(packet.read2ByteTxRx(sid, 11))[0]] for sid in cfg['servo_ids']]
        # Validate and clamp the entire test chunk before issuing any motor goals.
        raw_actions = prediction['actions'][:args.max_steps]
        targets = [clamp_ticks(action, limits) for action in raw_actions]
        current, _, _ = read_positions(packet, cfg)
        report.update(servo_limits=limits, initial_ticks=current, fps=prediction['fps'])
        write_targets(packet, cfg['servo_ids'], clamp_ticks(current, limits))
        powered = True
        set_torque(packet, torque_register, cfg['servo_ids'], True)
        deadline = time.monotonic()
        for index, (raw, target) in enumerate(zip(raw_actions, targets)):
            write_targets(packet, cfg['servo_ids'], target)
            deadline += 1 / prediction['fps']
            time.sleep(max(0, deadline-time.monotonic()))
            actual, _, _ = read_positions(packet, cfg)
            report['commands'].append(dict(step=index, prediction=raw,
                                           sent_ticks=target, measured_ticks=actual))
            print(f'Step {index+1}/{len(targets)}: sent={target} measured={actual}', flush=True)
    finally:
        try:
            if powered:
                print('Returning to saved start pose; leaving torque enabled to hold.', flush=True)
                return_to_pose(packet, torque_register, cfg, start_pose,
                               lambda: -1, keep_powered=True)
                report['returned_to_start'] = True
                report['torque_left_enabled'] = True
                report['final_ticks'] = read_positions(packet, cfg)[0]
        finally:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2)+'\n')
            if port is not None:
                port.closePort()
            if lock is not None:
                lock.close()


if __name__ == '__main__':
    main()
