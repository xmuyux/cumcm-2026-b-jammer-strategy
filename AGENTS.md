# Collaboration Notes

This repository contains the CUMCM 2026 Problem B model, strategy, and an offline simulator.

- Use `python experiment.py 30 5 q4_plain q4_early q4_early_opp` for offline comparisons.
- Use `python run.py --cases 20 --directional 5 --mode mixed` for offline evaluation.
- Do not run `run.py --real` unless the repository owner explicitly supplies the simulator endpoint, robot ID, and permission to use a formal or practice test.
- Do not commit simulator response logs, case codes, robot IDs, credentials, or local databases.
- Any directional-search change must retain and run the full-arena surround validation in `surround.py` before it is treated as safe.
