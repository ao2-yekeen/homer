#!/usr/bin/env python3
"""One-command GPU setup, resumable training, and return ZIP creation."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import venv
import zipfile

ROOT = Path(__file__).resolve().parent


def command(args):
    subprocess.run([str(arg) for arg in args], cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int)
    parser.add_argument("--batch-size", type=int)
    args = parser.parse_args()
    correction_run = (ROOT / "dataset.json").exists()
    args.steps = args.steps or (2000 if correction_run else 20000)
    if not (3, 10) <= sys.version_info[:2] <= (3, 12):
        raise SystemExit("Please run this with Python 3.10, 3.11 or 3.12.")
    environment = ROOT / ".venv"
    python = environment / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    ready = environment / "act-gpu-ready.json"
    if not ready.exists():
        print("[1/4] Creating local environment and installing GPU dependencies...", flush=True)
        if not python.exists():
            venv.EnvBuilder(with_pip=True).create(environment)
        command([python, "-m", "pip", "install", "--upgrade", "pip"])
        command([python, "-m", "pip", "install", "torch==2.7.1", "torchvision==0.22.1",
                 "--index-url", "https://download.pytorch.org/whl/cu128"])
        command([python, "-m", "pip", "install", "-r", ROOT / "requirements.txt"])
        ready.write_text(json.dumps({"torch": "2.7.1+cu128", "lerobot": "0.4.4"}))
    print("[2/4] Checking NVIDIA GPU...", flush=True)
    result = subprocess.run([str(python), "-c",
        "import json,torch; "
        "assert torch.cuda.is_available(), 'NVIDIA GPU unavailable. Install/update the NVIDIA driver and run again.'; "
        "print(json.dumps({'name':torch.cuda.get_device_name(0),'memory_gb':torch.cuda.get_device_properties(0).total_memory/2**30}))"],
        cwd=ROOT, check=True, capture_output=True, text=True)
    gpu = json.loads(result.stdout.strip().splitlines()[-1])
    batch_size = args.batch_size or (8 if gpu["memory_gb"] >= 10 else 2)
    print(f"GPU: {gpu['name']} ({gpu['memory_gb']:.1f} GB); batch size {batch_size}", flush=True)
    output = ROOT / "output"
    print("[3/4] Training ACT. Progress and ETA below; checkpoints every 200 steps.", flush=True)
    training = [python, "-u", ROOT / "train.py", "--data", ROOT / "data",
                "--output", output,
                "--device", "cuda", "--steps", args.steps, "--batch-size", batch_size,
                "--threads", min(8, os.cpu_count() or 2)]
    if correction_run:
        training.extend(["--dataset-manifest", ROOT / "dataset.json",
                         "--learning-rate", "0.00001"])
    else:
        training.extend(["--episodes-file", ROOT / "episodes.txt"])
    if (output / "latest" / "training.pt").exists():
        training.append("--resume")
    elif (output / "checkpoint-previous" / "training.pt").exists() and not (output / "latest").exists():
        (output / "checkpoint-previous").rename(output / "latest")
        training.append("--resume")
    if correction_run and "--resume" not in training:
        training.extend(["--init-from", ROOT / "initial-policy"])
    # Stream to the terminal and keep the same output in a portable text log.
    output.mkdir(exist_ok=True)
    with (output / "terminal.log").open("a", encoding="utf-8", buffering=1) as log:
        process = subprocess.Popen([str(arg) for arg in training], cwd=ROOT,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, encoding="utf-8", errors="replace")
        try:
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
            status = process.wait()
        except KeyboardInterrupt:
            # Console Ctrl+C reaches the child too; allow its checkpoint save to finish.
            print("Saving progress before exit. Run the same command to resume.", flush=True)
            process.wait()
            return
    if status:
        raise SystemExit(f"Training stopped (exit {status}). Details: output/terminal.log. "
                         "Run the same command to resume from the last saved checkpoint.")
    print("[4/4] Creating the results ZIP...", flush=True)
    checkpoint = output / ("best" if (output / "best").exists() else "latest")
    destination = ROOT / "act-trained-results.zip"
    with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
        for path in sorted(checkpoint.rglob("*")):
            if path.is_file() and path.name != "training.pt":
                archive.write(path, "policy/" + path.relative_to(checkpoint).as_posix())
        for name in ("dataset.json", "run.json", "metrics.jsonl", "terminal.log"):
            if (output / name).exists():
                archive.write(output / name, name)
        archive.writestr("README.txt", "Trained ACT policy, selected by held-out episode validation.\n"
                         "Inputs: wrist + overview RGB, joint state in raw ticks.\n"
                         "Actions: future absolute joint ticks, starting one frame ahead.\n"
                         "Use the included pre/post processors and dataset.json.\n"
                         "Offline training metrics do not establish physical grasp success.\n")
    print(f"DONE. Send back: {destination}\n"
          "The resumable optimizer checkpoint remains in output/latest on this PC.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        if error.stderr:
            print(error.stderr, file=sys.stderr)
        raise SystemExit(f"Setup failed (exit {error.returncode}). Fix the error above and run python run.py again.")
