#!/usr/bin/env python3
"""Run ACT on the PC, then open hand-guided correction recording on the Pi."""
import argparse
from datetime import datetime
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, default=ROOT / 'outputs/kaggle-trained/policy')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'outputs' / datetime.now().strftime('correction-demo-%Y%m%d-%H%M%S-%f'))
    parser.add_argument('--host', default='pi')
    parser.add_argument('--steps', type=int, default=342,
                        help='Policy motor steps before manual handover (default: 342, reduced from 392)')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Run from an interactive PC terminal for hand-guided correction keys')
    print(f'Running ACT for {args.steps} steps, then opening manual correction recording.', flush=True)
    subprocess.run([sys.executable, '-u', str(ROOT / 'training/run_policy.py'),
                    '--checkpoint', str(args.checkpoint), '--output', str(args.output),
                    '--host', args.host, '--steps', str(args.steps)], check=True)
    print('ACT stopped; arm holding. Correction recorder starting: '
          'support the arm, then press h to release torque and record.', flush=True)
    command = 'cd ~/mobile-robot && ' + shlex.join([
        'bash', 'act/scripts/start-corrections.sh', '--source-run', args.output.name])
    subprocess.run(['ssh', '-t', args.host, command], check=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode)
