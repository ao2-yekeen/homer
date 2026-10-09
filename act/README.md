# ACT grasping work

Keep all files for this ACT work inside `act/`. Run commands from the repository root.

- [General plan](docs/plan.md)
- [Step 2: hand-guided recording](docs/step-2.md)
- [Training setup and existing checkpoint check](training/README.md)

Layout:

```text
act/
  docs/       Plans
  training/   Training tools, dependency lock and source audit
  runtime/    Recorder, dataset export and policy execution (planned)
  config/     Rig configuration (planned)
  data/       Local recordings and datasets (ignored)
  outputs/    Local checkpoints and reports (ignored)
  .venv/      Local Python environment (ignored)
```

Reuse the existing arm SDK and limits without moving shared robot code.
Local rig settings belong in `act/config/rig.local.yaml`, which is ignored.
