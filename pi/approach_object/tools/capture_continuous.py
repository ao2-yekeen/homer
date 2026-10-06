#!/usr/bin/env python3
"""Keep RGB/ToF captures and check the calibration board until Ctrl+C."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import uuid

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parents[2]


def run(command):
    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or 'capture command failed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval', type=float, default=3, help='Seconds to wait after each capture (default: 3)')
    parser.add_argument('--square-mm', type=float, default=23, help='Printed square width (default: 23 mm)')
    parser.add_argument('--pi', default='pi', help='SSH host alias')
    parser.add_argument('--output', type=Path, default=REPO / 'captures' / 'current')
    parser.add_argument('--count', type=int, help='Stop after this many captures; default: until Ctrl+C')
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval < 0:
        parser.error('--interval must be finite and nonnegative')
    if not math.isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error('--square-mm must be finite and positive')
    if args.count is not None and args.count < 1:
        parser.error('--count must be positive')
    if args.pi.startswith('-') or any(c.isspace() for c in args.pi):
        parser.error('invalid SSH host alias')
    output = args.output.expanduser().resolve()
    total = usable = 0
    print(f'Saving to {output}. Squares: {args.square_mm:g} mm. Ctrl+C stops.', flush=True)
    try:
        while args.count is None or total < args.count:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            folder = output / (stamp + '-' + uuid.uuid4().hex[:6])
            print(f'Capturing {total + 1} — hold the board still...', flush=True)
            run([sys.executable, str(TOOLS / 'capture_current_rgb_tof.py'),
                 '--pi', args.pi, '--output', str(folder)])
            run([sys.executable, str(TOOLS / 'capture_checkerboard.py'),
                 '--capture', str(folder), '--square-mm', str(args.square_mm)])
            result = json.loads((folder / 'data' / 'checkerboard.json').read_text())
            total += 1
            usable += int(result['both_detected'])
            rgb = '54 corners' if result['observations']['rgb']['found'] else 'not detected'
            ir = '54 corners' if result['observations']['tof']['found'] else 'not detected'
            print(f'{folder.name}: RGB {rgb}, IR {ir}; usable pairs this run: {usable}/{total}', flush=True)
            if args.count is None or total < args.count:
                print(f'Move the board; next capture after {args.interval:g}s.', flush=True)
                time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\nStopped. Saved captures are preserved.', flush=True)
    except (RuntimeError, subprocess.TimeoutExpired, OSError, ValueError) as exc:
        print(f'Capture stopped: {exc}\nSaved captures are preserved.', file=sys.stderr)
        return 1
    print(f'Saved {total} checked pairs; {usable} have all corners in both cameras.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
