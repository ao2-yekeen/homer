# ACT grasping work

Keep all files for this ACT work inside `act/`. Run commands from the repository root.

- [Collect demos: commands and handover](docs/collect-demos.md)
- [General plan](docs/plan.md)
- [Step 2: hand-guided recording](docs/step-2.md)
- [Training setup and existing checkpoint check](training/README.md)

Layout:

```text
act/
  docs/       Plans
  training/   Training tools, dependency lock and source audit
  scripts/    Start and setup-check commands
  runtime/    Demo recorder and review; export/policy execution planned
  config/     Rig configuration
  data/       Local recordings and datasets (ignored)
  outputs/    Local checkpoints and reports (ignored)
  .venv/      Local Python environment (ignored)
```

Reuse the existing arm SDK without moving shared robot code. Recording uses raw
ticks; task-specific motion limits stay in the shared controller.
Local rig settings belong in `act/config/rig.local.yaml`, which is ignored.
