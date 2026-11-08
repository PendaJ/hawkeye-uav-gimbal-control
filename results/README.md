# Hawkeye experimental results

This directory contains selected simulation-based experiment logs associated with the Hawkeye *Drones* manuscript.

## Offline NPSO optimization: `npso/`

Six optimization configurations are provided, identified as Experiments 2–7:

| Experiment | Weights (MAE, volatility, second difference, zero crossings) |
| --- | --- |
| 2 — MAE | (1, 0, 0, 0) |
| 3 — MAE + Volatility | (1, 1, 0, 0) |
| 4 — MAE + ZC | (1, 0, 0, 1) |
| 5 — MAE + Second Difference | (1, 0, 1, 0) |
| 6 — Equal Weights | (1, 1, 1, 1) |
| 7 — Reweighted | (1, 0.5, 0.5, 1) |

Each folder provides:
- `configuration.json` — weight vector, seed, candidate budget and scenario
- `best_parameters.json` — lowest-fitness evaluated gains `kp`, `kd` and observer bandwidth `omega_o`
- `evaluations.csv` — the 220 recorded candidate evaluations, including `fitness`, `global_best_fitness`, `raw_metrics` and `normalized_metrics`

The six published `configuration.json` files identify **`world_mode: lateral`** and **`seed: 42`**. These are the definitive configuration values for these records; old launcher documentation that suggests tuning was done in `hold` mode is outdated for this dataset.

The global-best convergence plot can be reconstructed by plotting each CSV's `global_best_fitness` against `eval`. Entries `raw_metrics` and `normalized_metrics` are Python-dictionary string representations; parse with `ast.literal_eval`, **not** `eval`.

The metrics are normalized by `J/(J+c)`, where the pilot-derived constants are in `../experiments/config/normalizers_lateral_15s.json` in the repository root (see [normalization metadata](../experiments/config/normalizers_lateral_15s.json)). These constants were estimated from 10 pilot evaluations with seed 2026.

The raw metrics are image-domain error statistics, *not* gimbal angular acceleration. In particular, visual-error volatility is defined in pilot metadata as mean absolute radial first difference per valid vision observation. Refer to the source scripts for exact definitions.

## Repeatability: `repeatability/Exp6/`

The selected ten fixed-gain Experiment 6 trials will be added separately. These trials evaluate *closed-loop repeatability at fixed gains*, **not** repeated random-seed NPSO optimization. Do not interpret them as evidence of cross-seed optimization convergence.

## Reproducibility limitations

Published summaries and evaluation histories document the recorded optimization, but cannot by themselves reproduce the original Gazebo runs without the matching local environment, model weights, carrier-motion plugin, and complete launch/dependency configuration. See [reproducibility checklist](../docs/reproducibility.md).

No physical flight validation is implied by these simulation data.
