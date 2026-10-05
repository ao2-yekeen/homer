#!/usr/bin/env python3
"""Prepare workstation inference, capture existing Pi ROS streams, test a name.

No camera ownership, service restarts, motion publishers or head commands.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import uuid

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from approach_object.named_object_pin import (DEPENDENCIES, MODEL_SHA256, MODEL_URL, CLIP_REVISION,
    TEXT_ENCODER_SHA256, TEXT_ENCODER_URL, ensure_asset)


def run(args, **kwargs):
    print('+ ' + shlex.join(str(a) for a in args), flush=True)
    return subprocess.run([str(a) for a in args], check=True, **kwargs)


def prepare_environment(directory):
    python = directory / 'bin/python'
    if not python.is_file():
        creator = shutil.which('python3.12') or shutil.which('python3.11') or sys.executable
        run([creator, '-m', 'venv', directory])
    check = subprocess.run([str(python), '-c',
        'import importlib.metadata as m,json; print(json.dumps({p:m.version(p) for p in ' + repr(list(DEPENDENCIES)) + '}))'],
        capture_output=True, text=True)
    installed = json.loads(check.stdout) if check.returncode == 0 else {}
    clip_check = subprocess.run([str(python), '-c',
        'import importlib.metadata as m,json; print(json.loads(m.distribution("clip").read_text("direct_url.json") or "{}").get("vcs_info",{}).get("commit_id"))'],
        capture_output=True, text=True)
    if installed != DEPENDENCIES or clip_check.stdout.strip() != CLIP_REVISION:
        run([python, '-m', 'pip', 'install', 'torch==' + DEPENDENCIES['torch'],
             'torchvision==' + DEPENDENCIES['torchvision'], '--index-url', 'https://download.pytorch.org/whl/cpu'])
        run([python, '-m', 'pip', 'install', '-r', PACKAGE / 'requirements-yoloe-workstation.txt'])
    return python


def capture_pi(host, destination, frames, servo_deg=None):
    remote = '/tmp/homer-named-test-' + uuid.uuid4().hex
    with tempfile.TemporaryDirectory(prefix='homer-capture-upload-') as temporary:
        root = Path(temporary)
        source = root / 'approach_object'
        source.mkdir()
        (source / '__init__.py').touch()
        for name in ('model_io.py', 'model_pin.py'):
            shutil.copy2(PACKAGE / 'approach_object' / name, source / name)
        shutil.copy2(PACKAGE / 'tools/capture_rgb_tof.py', root / 'capture_rgb_tof.py')
        archive = root / 'code.tar'
        with tarfile.open(archive, 'w') as bundle:
            bundle.add(source, arcname='approach_object')
            bundle.add(root / 'capture_rgb_tof.py', arcname='capture_rgb_tof.py')
        with archive.open('rb') as upload:
            run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', host,
                 'mkdir -p ' + shlex.quote(remote) + ' && tar -xf - -C ' + shlex.quote(remote)], stdin=upload)
    # Reuse the network environment used by the working camera services.
    command = '\n'.join([
        'set -e', 'source /opt/ros/jazzy/setup.bash',
        'if [ -f "$HOME/.config/homer-slam/network/ros.env" ]; then',
        '  set -a; source "$HOME/.config/homer-slam/network/ros.env"; set +a',
        'else', '  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp', 'fi',
        'export PYTHONPATH=' + shlex.quote(remote) + ':"$PYTHONPATH"',
        'exec /usr/bin/python3 ' + shlex.quote(remote + '/capture_rgb_tof.py')
        + ' --auxiliary --frames ' + str(frames) + ' --output ' + shlex.quote(remote + '/capture')
        + ('' if servo_deg is None else ' --servo-deg ' + str(servo_deg)),
    ])
    run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', host, command])
    destination.parent.mkdir(parents=True, exist_ok=True)
    run(['scp', '-r', host + ':' + remote + '/capture', destination])
    return remote


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', required=True)
    parser.add_argument('--pi', default='pi')
    parser.add_argument('--frames', type=int, default=3)
    parser.add_argument('--capture', type=Path, help='reuse saved capture instead of taking fresh frames')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--environment', type=Path)
    parser.add_argument('--calibration', type=Path, default=PACKAGE / 'config/calibration.yaml')
    parser.add_argument('--servo-deg', type=float)
    parser.add_argument('--urdf', type=Path, help='robot model for unqualified contact geometry checks')
    parser.add_argument('--nominal', action='store_true')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--reference-clearance-m', type=float)
    parser.add_argument('--nominal-depth-semantics', choices=('radial', 'optical_z'), default='radial')
    args = parser.parse_args()
    if not args.target.strip() or not 1 <= args.frames <= 100 or not 320 <= args.imgsz <= 1280 or args.imgsz % 32:
        parser.error('nonempty target, 1–100 frames and image size 320–1280 in multiples of 32 required')
    if args.pi.startswith('-') or any(character.isspace() for character in args.pi):
        parser.error('invalid SSH host alias')
    if args.reference_clearance_m is not None and (not math.isfinite(args.reference_clearance_m) or args.reference_clearance_m < 0):
        parser.error('reference clearance must be finite and nonnegative')
    if args.servo_deg is not None and (not math.isfinite(args.servo_deg) or not 80 <= args.servo_deg <= 150):
        parser.error('known servo command must be within 80–150 degrees')
    cache = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache'))) / 'homer'
    trial = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:6]
    output = (args.output or cache / 'named-object-tests' / trial).resolve()
    if output.exists():
        parser.error('output directory already exists; choose a fresh trial destination')
    python = prepare_environment((args.environment or cache / 'yoloe-venv').resolve())
    model = (args.model or cache / 'models/yoloe-26n-seg.pt').resolve()
    ensure_asset(model, MODEL_URL, MODEL_SHA256)
    ensure_asset(model.parent / 'mobileclip2_b.ts', TEXT_ENCODER_URL, TEXT_ENCODER_SHA256)
    output.mkdir(parents=True)
    capture = args.capture.resolve() if args.capture else output / 'capture'
    remote = None if args.capture else capture_pi(args.pi, capture, args.frames, args.servo_deg)
    command = [python, PACKAGE / 'tools/named_object_distance_test.py', '--target', args.target,
               '--model', model, '--capture', capture, '--calibration', args.calibration.resolve(),
               '--output', output / 'diagnostic', '--imgsz', str(args.imgsz)]
    if args.servo_deg is not None:
        command.extend(['--servo-deg', str(args.servo_deg)])
    if args.urdf:
        command.extend(['--urdf', str(args.urdf.resolve())])
    if args.nominal:
        command.extend(['--nominal', '--nominal-depth-semantics', args.nominal_depth_semantics])
    if args.reference_clearance_m is not None:
        command.extend(['--reference-clearance-m', str(args.reference_clearance_m)])
    # Keep the downloaded text encoder in the persistent model cache.
    run(command, cwd=model.parent)
    (output / 'run.json').write_text(json.dumps(dict(target=args.target, capture=str(capture),
        remote_capture_root=remote, result=str(output / 'diagnostic/result.json'), motion_commanded=False), indent=2) + '\n')
    print('Saved result: ' + str(output / 'diagnostic/result.json'), flush=True)


if __name__ == '__main__':
    main()
