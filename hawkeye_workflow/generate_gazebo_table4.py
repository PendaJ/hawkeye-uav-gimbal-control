#!/usr/bin/env python3

import csv
import math
from pathlib import Path

BASE = Path.home() / "hawkeye_x3_ws/rosbags/hawkeye_workflow"
OUT = Path.home() / "hawkeye_x3_ws/paper_results/Gazebo_Figure8_Scenario2"

# Experiment, fitness, run folder, initial x, initial y
RUNS = [
    (2, "MAE", "hold_custom_20261106_175007",
     -195.977890, -145.021912),

    (3, "MAE + Vol.", "hold_custom_20261106_175558",
     -195.635269, -145.035706),

    (4, "MAE + ZC", "hold_custom_20261106_191132",
     -196.256256, -145.176544),

    (5, "MAE + Acc.", "hold_custom_20261106_191356",
     -196.292343, -145.622498),

    (6, "Combined (equal weights)", "hold_custom_20261106_191618",
     -196.232056, -145.109192),

    (7, "Combined (reweighted)", "hold_custom_20261106_191805",
     -196.134987, -145.123413),
]


def compute_overshoot(values, initial):
    sign = 1 if initial > 0 else -1
    return max(0.0, max(-sign * v for v in values))


results = []

for exp, fitness, folder, ex0, ey0 in RUNS:

    path = BASE / folder / "analysis/tracking.csv"

    if not path.is_file():
        raise FileNotFoundError(f"Missing: {path}")

    with path.open(newline="") as f:
        data = list(csv.DictReader(f))

    samples = []

    for row in data:
        t = float(row["time_from_control_start_s"])
        ex = float(row["ex_px"])
        ey = float(row["ey_px"])

        if (0 <= t <= 5.0 and
                all(math.isfinite(v) for v in (t, ex, ey))):
            samples.append((t, ex, ey))

    if not samples:
        raise ValueError(f"No valid samples: Experiment {exp}")

    ox = compute_overshoot([r[1] for r in samples], ex0)
    oy = compute_overshoot([r[2] for r in samples], ey0)

    if ox >= oy:
        maximum = ox
        initial_reference = abs(ex0)
        axis = "x"
    else:
        maximum = oy
        initial_reference = abs(ey0)
        axis = "y"

    percentage = 100 * maximum / initial_reference

    results.append({
        "Exp": exp,
        "Fitness": fitness,
        "Overshoot_px": maximum,
        "Overshoot_percent": percentage,
        "Worst_axis": axis
    })


baseline = results[0]["Overshoot_px"]

if baseline <= 0:
    raise ValueError("Experiment 2 has zero overshoot.")

OUT.mkdir(parents=True, exist_ok=True)

latex = [
    r"\begin{table}[t]",
    r"\centering",
    r"\caption{Scenario 2 overshoot results in Gazebo for Experiments 2--7.}",
    r"\label{tab:scenario2_overshoot}",
    r"\begin{tabular}{clrrr}",
    r"\hline",
    r"Exp. & Fitness function & Overshoot [px] & "
    r"Overshoot [\%] & Change vs. Exp. 2 \\",
    r"\hline"
]

print("\n=== NEW TABLE 4: GAZEBO RESULTS ===\n")
print(f"{'Exp':<5} {'Fitness':<27} {'Overshoot px':>14} "
      f"{'Overshoot %':>14} {'Change':>12}")

for r in results:

    change = 100 * (r["Overshoot_px"] - baseline) / baseline
    change_text = "--" if r["Exp"] == 2 else f"{change:+.1f}%"
    change_tex = "--" if r["Exp"] == 2 else f"{change:+.1f}\\%"

    r["Change_vs_Exp2_percent"] = change

    print(f'{r["Exp"]:<5} {r["Fitness"]:<27} '
          f'{r["Overshoot_px"]:14.2f} '
          f'{r["Overshoot_percent"]:14.2f} '
          f'{change_text:>12}')

    latex.append(
        f'{r["Exp"]} & {r["Fitness"]} & '
        f'{r["Overshoot_px"]:.2f} & '
        f'{r["Overshoot_percent"]:.2f} & '
        f'{change_tex} ' + r'\\'
    )

latex += [
    r"\hline",
    r"\end{tabular}",
    r"\end{table}"
]

tex_file = OUT / "table4_gazebo.tex"
csv_file = OUT / "table4_gazebo.csv"

tex_file.write_text("\n".join(latex), encoding="utf-8")

with csv_file.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)

print("\n=== LATEX TABLE FOR OVERLEAF ===\n")
print("\n".join(latex))

print("\nSaved:", tex_file)
print("Saved:", csv_file)
