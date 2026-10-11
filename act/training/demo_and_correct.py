#!/usr/bin/env python3
"""Run a fresh ACT attempt for each manual correction."""
import argparse
import json
from datetime import datetime
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def saved_chunk_pose(directory, steps_before_end=20):
    """Select measured feedback N motor steps before the saved run ended."""
    report_path = directory / 'movement-report.json'
    commands = json.loads(report_path.read_text())['commands']
    steps = max(0, len(commands) - steps_before_end)
    states = [json.loads(path.read_text()) for path in directory.glob('*-state.json')]
    initial = min(states, key=lambda state: state['executed_steps'])
    ticks = commands[steps-1]['measured_ticks'] if steps else initial['state_ticks']
    return dict(state_ticks=ticks, joint_names=initial['joint_names'],
                source_run=directory.name, source_state=report_path.name,
                executed_steps=steps, steps_before_end=steps_before_end,
                initial_ticks=initial['state_ticks'],
                replay_actions=[command['sent_ticks'] for command in commands[:steps]])


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
    parser.add_argument('--source-run', type=Path, help='Replay this saved PC policy run instead of fresh predictions each demo')
    parser.add_argument('--steps-before-end', type=int, default=20,
                        help='Correction pose this many motor steps before the end (default: 20)')
    parser.add_argument('--run-act', action='store_true',
                        help='Use fresh ACT attempts (the default unless --source-run is supplied)')
    args = parser.parse_args()
    if args.steps_before_end < 0:
        parser.error('--steps-before-end must be zero or greater')
    if not sys.stdin.isatty():
        parser.error('Run from an interactive PC terminal for hand-guided correction keys')
    if args.run_act or args.source_run is None:
        attempt = 0
        while True:
            source = args.output if attempt == 0 else args.output.with_name(f'{args.output.name}-{attempt+1}')
            steps = max(1, args.steps - args.steps_before_end)
            print(f'Running a fresh ACT attempt for {steps} steps, then holding for h.', flush=True)
            subprocess.run([sys.executable, '-u', str(ROOT / 'training/run_policy.py'),
                            '--checkpoint', str(args.checkpoint), '--output', str(source),
                            '--host', args.host, '--steps', str(steps)], check=True)
            command = 'cd ~/mobile-robot && ' + shlex.join([
                'bash', 'act/scripts/start-corrections.sh', '--source-run', source.name, '--once'])
            result = subprocess.run(['ssh', '-t', args.host, command], check=False)
            if result.returncode == 0:
                return
            if result.returncode != 10:
                raise subprocess.CalledProcessError(result.returncode, result.args)
            if input('Saved. Reset object/container. Enter = fresh demo; q = quit: ').strip().lower() == 'q':
                return
            attempt += 1
    source = args.source_run
    pose = saved_chunk_pose(source, args.steps_before_end)
    pose_file = source / 'correction-start-pose.json'
    pose_file.write_text(json.dumps(pose, indent=2)+'\n')
    remote_pose = 'act/data/correction-start-pose.json'
    subprocess.run(['scp', str(pose_file),
                    f'{args.host}:mobile-robot/{remote_pose}'], check=True)
    print(f'Corrections from {source.name}, step {pose["executed_steps"]}: '
          f'{pose["state_ticks"]}. Startup returns to the initial pose and replays '
          'the saved approach before asking for h. After saving, it returns '
          'directly to the correction pose.', flush=True)
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
