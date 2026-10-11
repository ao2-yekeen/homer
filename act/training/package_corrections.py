#!/usr/bin/env python3
"""Package prepared correction data and the existing ACT policy for a friend."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', type=Path, default=ROOT.parent / 'outputs/correction-training')
    parser.add_argument('--checkpoint', type=Path, default=ROOT.parent / 'outputs/kaggle-trained/policy')
    parser.add_argument('--output', type=Path, default=ROOT.parent / 'outputs/act-correction-training.zip')
    args = parser.parse_args()
    manifest = json.loads((args.prepared / 'dataset.json').read_text())
    manifest['init_from'] = 'initial-policy'
    prefix = 'act-correction-training/'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix('.writing.zip')
    with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_STORED, allowZip64=True) as archive:
        archive.writestr(prefix + 'dataset.json', json.dumps(manifest, indent=2) + '\n')
        for name in ('preparation.json', 'verification.json'):
            archive.write(args.prepared / name, prefix + name)
        names = manifest['train'] + manifest['validation']
        for count, name in enumerate(names, 1):
            directory = args.prepared / 'data' / name
            for path in sorted(directory.rglob('*')):
                if path.is_file():
                    archive.write(path, prefix + 'data/' + name + '/' + path.relative_to(directory).as_posix())
            print(f'Packaged {count}/{len(names)} demos', flush=True)
        for path in sorted(args.checkpoint.rglob('*')):
            if path.is_file() and path.name != 'training.pt':
                archive.write(path, prefix + 'initial-policy/' + path.relative_to(args.checkpoint).as_posix())
        for source, target in [('run.py', 'run.py'), ('train.py', 'train.py'),
                               ('requirements-gpu.txt', 'requirements.txt'),
                               ('CORRECTION-FRIEND-README.md', 'README.md'),
                               ('start.sh', 'start.sh'), ('start.ps1', 'start.ps1'),
                               ('start.bat', 'start.bat')]:
            archive.write(ROOT / source, prefix + target)
    temporary.replace(args.output)
    digest = hashlib.sha256()
    with args.output.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    args.output.with_suffix('.zip.sha256').write_text(digest.hexdigest() + '  ' + args.output.name + '\n')
    print(f'Ready: {args.output} ({args.output.stat().st_size / 2**20:.1f} MiB)', flush=True)


if __name__ == '__main__':
    main()
