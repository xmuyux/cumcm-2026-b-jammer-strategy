# Team Handoff

## Setup

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

## Safe starting points

```powershell
# Offline Q4 comparison only; no account or network access is used.
python experiment.py 30 5 q4_plain q4_early q4_early_opp

# Offline random-case evaluation.
python run.py --cases 20 --directional 5 --mode mixed

# Verify the directional-source surround condition for candidate lattices.
python design_search.py
```

`MockArena` in `arena.py` is entirely local. The formal simulator client is isolated behind `run.py --real`; never use it without explicit authorisation and the required connection details.

## Repository contents

- `strategy.py`: main search, localisation, and clearing policy.
- `arena.py`, `geometry.py`, `surround.py`: offline rules and proof-related geometry.
- `experiment.py`, `design_search.py`, `optimize_design.py`: repeatable comparisons and search-pattern design tools.
- `solution_summary.md`, `q12_results.md`, and the figures: modelling notes and results.
