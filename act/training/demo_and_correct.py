#!/usr/bin/env python3
"""Start repeatable corrections from a saved final-chunk pose, without rerunning ACT."""
import argparse
import json
from datetime import datetime
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def saved_chunk_pose(directory):
    """Use measured feedback entering the final chunk, never a predicted goal."""
    states = [(path, json.loads(path.read_text())) for path in directory.glob('*-state.json')]
    path, state = max(((path, state) for path, state in states if not state.get('finished')),
                      key=lambda item: item[1]['executed_steps'])
    return dict(state_ticks=state['state_ticks'], joint_names=state['joint_names'],
                source_run=directory.name, source_state=path.name,
                executed_steps=state['executed_steps'])


def latest_completed_run():
    runs = sorted((ROOT / 'outputs').glob('*/movement-report.json'),
                  key=lambda path: path.stat().st_mtime, reverse=True)
    for report in runs:
        if json.loads(report.read_text()).get('completed') and list(report.parent.glob('*-state.json')):
            return report.parent
    raise RuntimeError('No completed ACT run found; provide --source-run PATH or use --run-act')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, default=ROOT / 'outputs/kaggle-trained/policy')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'outputs' / datetime.now().strftime('correction-demo-%Y%m%d-%H%M%S-%f'))
    parser.add_argument('--host', default='pi')
    parser.add_argument('--steps', type=int, default=342,
                        help='Policy motor steps before manual handover (default: 342, reduced from 392)')
    parser.add_argument('--source-run', type=Path, help='Existing PC policy output directory')
    parser.add_argument('--run-act', action='store_true',
                        help='Run a fresh ACT attempt before corrections instead of skipping it')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Run from an interactive PC terminal for hand-guided correction keys')
    if args.run_act:
        print(f'Running ACT for {args.steps} steps before corrections.', flush=True)
        subprocess.run([sys.executable, '-u', str(ROOT / 'training/run_policy.py'),
                        '--checkpoint', str(args.checkpoint), '--output', str(args.output),
                        '--host', args.host, '--steps', str(args.steps)], check=True)
        source = args.output
    else:
        source = args.source_run or latest_completed_run()
    pose = saved_chunk_pose(source)
    pose_file = source / 'correction-start-pose.json'
    pose_file.write_text(json.dumps(pose, indent=2)+'\n')
    remote_pose = 'act/data/correction-start-pose.json'
    subprocess.run(['scp', str(pose_file),
                    f'{args.host}:mobile-robot/{remote_pose}'], check=True)
    print(f'Corrections from {source.name}, step {pose["executed_steps"]}: '
          f'{pose["state_ticks"]}. Reset the object to match this pose. '
          'Arm moves on startup and after each saved correction.', flush=True)
    command = 'cd ~/mobile-robot && ' + shlex.join([
        'bash', 'act/scripts/start-corrections.sh', '--source-run', source.name,
        '--start-pose', remote_pose])
    subprocess.run(['ssh', '-t', args.host, command], check=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode)
