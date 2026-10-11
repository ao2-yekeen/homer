#!/usr/bin/env python3
"""Run a full policy attempt: inference here, cameras/motors on the Pi."""
import argparse
import base64
import contextlib
import io
import json
from pathlib import Path
import shlex
import subprocess

from PIL import Image

from predict_snapshot import SnapshotPolicy


def close_session(process):
    """Close the protocol and reap SSH, including after Ctrl+C."""
    with contextlib.suppress(BrokenPipeError):
        process.stdin.close()
    try:
        return process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--host', default='pi')
    parser.add_argument('--steps', type=int, default=392)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    policy = SnapshotPolicy(args.checkpoint)
    remote_output = 'act/outputs/' + args.output.name
    subprocess.run(['ssh', args.host, 'systemctl --user stop soarm-bridge.path soarm-bridge robot-teleop rgb-object-detection'], check=True)
    command = 'cd ~/mobile-robot && python3 -u act/runtime/policy_session.py ' + shlex.join([
        '--config', 'act/config/rig.local.yaml', '--output', remote_output, '--steps', str(args.steps)])
    process = subprocess.Popen(['ssh', '-o', 'ServerAliveInterval=5', '-o',
                                'ServerAliveCountMax=3', args.host, command], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, text=True, bufsize=1,
                               start_new_session=True)
    chunk = 0
    try:
        for line in process.stdout:
            snapshot = json.loads(line)
            images = {}
            for name, encoded in snapshot.pop('images').items():
                raw = base64.b64decode(encoded)
                (args.output / f'{chunk:03d}-{name}.jpg').write_bytes(raw)
                images[name] = Image.open(io.BytesIO(raw))
            (args.output / f'{chunk:03d}-state.json').write_text(json.dumps(snapshot, indent=2)+'\n')
            if snapshot.get('finished'):
                for image in images.values():
                    image.close()
                print(f'Finished {snapshot["executed_steps"]} motor steps; arm holding final position.', flush=True)
                break
            try:
                prediction = policy.predict(snapshot['state_ticks'], images)
            finally:
                for image in images.values():
                    image.close()
            (args.output / f'{chunk:03d}-prediction.json').write_text(json.dumps(prediction, indent=2)+'\n')
            print(f'Chunk {chunk+1}: {snapshot["executed_steps"]}/{args.steps} steps executed; '
                  f'joints={snapshot["state_ticks"]}; inference={prediction["inference_seconds"]:.2f}s', flush=True)
            process.stdin.write(json.dumps(prediction)+'\n')
            process.stdin.flush()
            chunk += 1
    finally:
        # Closing input also stops the Pi during its initial-pose wait.
        status = close_session(process)
        subprocess.run(['scp', f'{args.host}:mobile-robot/{remote_output}/movement-report.json',
                        str(args.output / 'movement-report.json')], check=False)
        subprocess.run(['ssh', args.host, 'systemctl --user start rgb-object-detection'], check=False)
    if status:
        raise SystemExit(f'Pi execution failed: exit {status}')


if __name__ == '__main__':
    main()
