#!/usr/bin/env python3
"""Build a portable ZIP with training scripts and successful recorded demos."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import sys
import zipfile

from PIL import Image

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'runtime'))
from trim_handover import trim_handover


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=ROOT.parent / "data")
    parser.add_argument("--episodes-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT.parent / "outputs/act-gpu-training.zip")
    args = parser.parse_args()
    names = [line.strip().rstrip("/") for line in args.episodes_file.read_text().splitlines() if line.strip()]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(".writing.zip")
    frames = 0
    removed_frames = 0
    with zipfile.ZipFile(temporary, "w", zipfile.ZIP_STORED, allowZip64=True) as archive:
        archive.writestr("act-gpu-training/episodes.txt", "\n".join(names) + "\n")
        for count, name in enumerate(names, 1):
            directory = args.data / name
            metadata = json.loads((directory / "metadata.json").read_text())
            if metadata["outcome"] != "success" or metadata.get("joint_only"):
                raise ValueError(f"Not a successful camera demo: {name}")
            compressed = directory / "samples.jsonl.gz"
            with (gzip.open(compressed, "rt") if compressed.exists()
                  else (directory / "samples.jsonl").open()) as stream:
                rows = [json.loads(line) for line in stream]
            if len(rows) != metadata["samples"]:
                raise ValueError(f"Incomplete samples: {name}")
            rows, metadata, removed = trim_handover(rows, metadata)
            removed_frames += len(removed)
            prefix = "act-gpu-training/data/" + name + "/"
            for row in rows:
                for field in ("image", "overview_image"):
                    source = directory / row[field]
                    target = Path(row[field]).with_suffix(".jpg").name
                    with Image.open(source) as image:
                        rgb = image.convert("RGB").resize((320, 240), Image.Resampling.BILINEAR)
                        encoded = io.BytesIO()
                        rgb.save(encoded, format="JPEG", quality=75)
                    archive.writestr(prefix + target, encoded.getvalue())
                    row[field] = target
            metadata["package_export"] = {"original_image_size": [metadata["config"]["width"],
                                                                       metadata["config"]["height"]],
                                          "image_size": [320, 240], "jpeg_quality": 75}
            metadata["config"]["width"] = 320
            metadata["config"]["height"] = 240
            metadata["config"]["overview_camera"].update(width=320, height=240)
            archive.writestr(prefix + "metadata.json", json.dumps(metadata, indent=2))
            contents = "".join(json.dumps(row) + "\n" for row in rows).encode()
            archive.writestr(prefix + "samples.jsonl.gz", gzip.compress(contents, mtime=0))
            frames += len(rows)
            print(f"Packaged {count}/{len(names)} demos ({frames:,} frames; "
                  f"{removed_frames:,} idle handover frames removed)", flush=True)
        for source, target in (("run.py", "run.py"), ("train.py", "train.py"),
                               ("requirements-gpu.txt", "requirements.txt"),
                               ("package-readme.md", "README.md"), ("start.sh", "start.sh"),
                               ("start.bat", "start.bat"), ("start.ps1", "start.ps1")):
            archive.write(ROOT / source, "act-gpu-training/" + target)
        archive.writestr("act-gpu-training/package.json", json.dumps({
            "episodes": len(names), "frames": frames, "image_size": [320, 240],
            "handover_frames_removed": removed_frames,
            "source": "Successful Raspberry Pi ACT recordings", "excluded": ["discard", "invalid", "source"]}, indent=2))
    temporary.replace(args.output)
    digest = hashlib.sha256()
    with args.output.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    args.output.with_suffix(".zip.sha256").write_text(digest.hexdigest() + "  " + args.output.name + "\n")
    print(f"Ready: {args.output} ({args.output.stat().st_size / 2**20:.1f} MiB)", flush=True)


if __name__ == "__main__":
    main()
