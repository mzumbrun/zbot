# Runbooks

Short "do this when…" procedures for Zbot: the steps that are easy to forget because they only
come up at certain times (after a calibration, on a new laptop, after a fault).

| File | Covers |
|---|---|
| [so101_arm.md](so101_arm.md) | SO-101 follower: daily start/stop, enabling and commanding, after every calibration, faults |
| [dev_env.md](dev_env.md) | Laptops, venv vs ROS terminals, building, keeping Victus and Upc in sync |

## Conventions

- **One file per subject** (a robot subsystem or a tool). Add a section to an existing file when
  the procedure belongs to that subject; start a new file when it doesn't, and add it to the
  table above.
- **Each section starts with its trigger**, i.e. *when* you do it ("After every
  `lerobot-calibrate`"), then numbered steps with copy-pasteable commands.
- Commands assume the repo at `~/zbot_ws/src/zbot` unless a step says otherwise.
- Explanations live elsewhere (`docs/arm_tuning.md`, script docstrings); runbooks just say what
  to do and link to the why.
- Update the "Last updated" line at the bottom of a file when you change it.
