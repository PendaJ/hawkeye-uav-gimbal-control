from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

BASE = Path.home() / "hawkeye_x3_ws/rosbags/hawkeye_workflow"

RUNS = {
    "ZN-PD": [
        "hold_pd_custom_20261107_002028",
        "hold_pd_custom_20261107_002205",
        "hold_pd_custom_20261107_002234",
    ],
    "ZN-SSPD": [
        "hold_custom_20261107_002102",
        "hold_custom_20261107_002134",
        "hold_custom_20261107_002303",
    ],
}

COLORS = {"ZN-PD": "tab:orange", "ZN-SSPD": "tab:blue"}

TOLERANCE = 2.0   # px
DISPLAY_END = 8.0 # seconds
GRID = np.arange(0, DISPLAY_END + 0.001, 0.05)

fig, ax = plt.subplots(figsize=(8, 4.5))
results = []

for controller, folders in RUNS.items():
    curves = []

    for index, folder in enumerate(folders, 1):
        path = BASE / folder / "analysis/tracking.csv"
        df = pd.read_csv(path)

        t = df["time_from_control_start_s"].to_numpy(dtype=float)
        ex = df["ex_px"].to_numpy(dtype=float)

        valid = np.isfinite(t) & np.isfinite(ex)
        t, ex = t[valid], ex[valid]

        order = np.argsort(t)
        t, ex = t[order], ex[order]

        # Remove duplicate timestamps.
        t, unique_idx = np.unique(t, return_index=True)
        ex = ex[unique_idx]

        if len(t) < 3:
            raise ValueError(f"Insufficient samples: {folder}")

        # Initial error direction.
        initial_error = ex[0]
        direction = np.sign(initial_error)

        if direction == 0:
            raise ValueError(f"Zero initial error: {folder}")

        oriented = direction * ex

        # Find first observed crossing of the reference.
        crossings = np.flatnonzero(oriented <= 0)

        overshoot = np.nan
        crossing_time = np.nan

        if len(crossings) > 0:
            first = crossings[0]
            crossing_time = t[first]

            # Largest opposite-side excursion after crossing.
            overshoot = max(0.0, -np.min(oriented[first:]))

        # Settling time over the ENTIRE recorded response.
        settling_time = np.nan

        for j in range(len(t)):
            if np.all(np.abs(ex[j:]) <= TOLERANCE):
                settling_time = t[j]
                break

        # Interpolate only within the observed time domain.
        interp = np.full_like(GRID, np.nan, dtype=float)
        mask = (GRID >= t.min()) & (GRID <= t.max())
        interp[mask] = np.interp(GRID[mask], t, ex)
        curves.append(interp)

        results.append({
            "Controller": controller,
            "Trial": index,
            "Initial error (px)": initial_error,
            "First crossing (s)": crossing_time,
            "Overshoot (px)": overshoot,
            "Settling time (s)": settling_time,
        })

        ax.plot(
            t[t <= DISPLAY_END],
            ex[t <= DISPLAY_END],
            color=COLORS[controller],
            alpha=0.20,
            lw=1.0,
        )

    curves = np.array(curves)
    count = np.sum(np.isfinite(curves), axis=0)

    mean_curve = np.full_like(GRID, np.nan)
    supported = count == len(folders)
    mean_curve[supported] = np.mean(curves[:, supported], axis=0)

    ax.plot(
        GRID,
        mean_curve,
        color=COLORS[controller],
        linewidth=2.5,
        label=controller,
    )

ax.axhline(0, color="black", ls="--", lw=0.9)
ax.axhline(TOLERANCE, color="gray", ls=":", lw=0.8)
ax.axhline(-TOLERANCE, color="gray", ls=":", lw=0.8)

ax.set_xlabel("Time (s)")
ax.set_ylabel("Horizontal tracking error (px)")
ax.set_xlim(0, DISPLAY_END)
ax.grid(alpha=0.25)
ax.legend()
fig.tight_layout()

fig.savefig("figure6_gazebo_response.pdf", bbox_inches="tight")
fig.savefig("figure6_gazebo_response.png",
            dpi=300, bbox_inches="tight")

summary = pd.DataFrame(results)

print("\n=== INDIVIDUAL TRIAL RESULTS ===")
print(summary.round(3).to_string(index=False))

print("\n=== PEAK OVERSHOOT SUMMARY ===")
print(
    summary.groupby("Controller")["Overshoot (px)"]
    .agg(["count", "mean", "std", "min", "max"])
    .round(3)
    .to_string()
)

print("\n=== SETTLING TIME SUMMARY ===")
print(
    summary.groupby("Controller")["Settling time (s)"]
    .agg(["count", "mean", "std"])
    .round(3)
    .to_string()
)

summary.to_csv("figure6_gazebo_metrics.csv", index=False)

plt.show()
