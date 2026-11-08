from pathlib import Path
import shutil
import numpy as np
import pandas as pd
import matplotlib

# Save PDF without requiring a graphical display
matplotlib.use("Agg")

import matplotlib.pyplot as plt

# ==========================================
# 1. LATEX / PUBLICATION FORMATTING
# ==========================================

USE_TEX = shutil.which("latex") is not None

plt.rcParams.update({
    "text.usetex": USE_TEX,
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "font.size": 10,
    "axes.labelsize": 11,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "axes.linewidth": 0.8,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

# ==========================================
# 2. GAZEBO EXPERIMENT RECORDINGS
# ==========================================

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

COLORS = {
    "ZN-PD": "red",
    "ZN-SSPD": "green",
}

# ==========================================
# 3. PLOT CONFIGURATION
# ==========================================

DISPLAY_END = 5.0
TOLERANCE = 2.0
GRID = np.arange(0, DISPLAY_END + 0.001, 0.01)

fig, ax = plt.subplots(figsize=(6.4, 3.5))

results = {}

# ==========================================
# 4. LOAD AND PLOT EXPERIMENTS
# ==========================================

for controller, folders in RUNS.items():

    curves = []
    overshoots = []

    for folder in folders:

        file = BASE / folder / "analysis/tracking.csv"

        if not file.exists():
            raise FileNotFoundError(file)

        df = pd.read_csv(file)

        t = df["time_from_control_start_s"].to_numpy(
            dtype=float
        )
        ex = df["ex_px"].to_numpy(dtype=float)

        valid = np.isfinite(t) & np.isfinite(ex)
        t = t[valid]
        ex = ex[valid]

        order = np.argsort(t)
        t = t[order]
        ex = ex[order]

        t, indices = np.unique(t, return_index=True)
        ex = ex[indices]

        if len(t) < 3:
            raise ValueError(
                f"Insufficient tracking samples: {folder}"
            )

        # Interpolate within the recorded time interval
        interp = np.full_like(GRID, np.nan)

        mask = (
            (GRID >= t.min()) &
            (GRID <= t.max())
        )

        interp[mask] = np.interp(
            GRID[mask], t, ex
        )

        curves.append(interp)

        # Calculate peak overshoot from full recording
        direction = np.sign(ex[0])
        oriented = direction * ex

        crossings = np.flatnonzero(oriented <= 0)

        overshoot = np.nan

        if len(crossings):
            first = crossings[0]
            overshoot = max(
                0.0,
                -np.min(oriented[first:])
            )

        overshoots.append(overshoot)

        # Plot individual trials using thin lines
        display = t <= DISPLAY_END

        ax.plot(
            t[display],
            ex[display],
            color=COLORS[controller],
            linewidth=0.8,
            alpha=0.25,
        )

    # Mean response
    curves = np.array(curves)

    supported = np.all(
        np.isfinite(curves), axis=0
    )

    mean_curve = np.full_like(GRID, np.nan)

    mean_curve[supported] = np.mean(
        curves[:, supported], axis=0
    )

    ax.plot(
        GRID,
        mean_curve,
        color=COLORS[controller],
        linewidth=1.8,
        label=controller,
    )

    results[controller] = {
        "mean": np.nanmean(overshoots),
        "std": np.nanstd(overshoots, ddof=1),
    }

# ==========================================
# 5. FIGURE FORMATTING
# ==========================================

ax.axhline(
    0,
    color="black",
    linestyle="--",
    linewidth=0.8,
)

ax.axhline(
    TOLERANCE,
    color="gray",
    linestyle=":",
    linewidth=0.6,
)

ax.axhline(
    -TOLERANCE,
    color="gray",
    linestyle=":",
    linewidth=0.6,
)

ax.set_xlim(0, DISPLAY_END)
ax.set_ylim(-10, 82)

ax.set_xlabel(r"Time (s)")
ax.set_ylabel(
    r"Horizontal tracking error, $e_x$ (px)"
)

ax.legend(
    loc="upper right",
    frameon=True,
    edgecolor="black",
    fancybox=False,
)

ax.tick_params(direction="in")
ax.grid(False)

# Overshoot annotation
pd_os = results["ZN-PD"]["mean"]
sspd_os = results["ZN-SSPD"]["mean"]

annotation = (
    rf"ZN-PD overshoot: ${pd_os:.2f}\,\mathrm{{px}}$"
    "\n"
    rf"ZN-SSPD overshoot: ${sspd_os:.2f}\,\mathrm{{px}}$"
)

ax.text(
    0.98,
    0.43,
    annotation,
    transform=ax.transAxes,
    ha="right",
    va="center",
    fontsize=8.5,
    bbox={
        "facecolor": "white",
        "edgecolor": "black",
        "boxstyle": "round,pad=0.3",
        "linewidth": 0.7,
    },
)

fig.tight_layout()

# ==========================================
# 6. SAVE VECTOR PDF
# ==========================================

output_pdf = Path("figure6_gazebo_tex.pdf")

try:
    fig.savefig(
        output_pdf,
        format="pdf",
        bbox_inches="tight",
    )
except Exception as exc:
    if USE_TEX:
        print("LaTeX rendering failed; using Matplotlib mathtext.")
        print(str(exc))
        plt.rcParams["text.usetex"] = False
        fig.savefig(
            output_pdf,
            format="pdf",
            bbox_inches="tight",
        )
    else:
        raise

# Also save PNG for inspection
fig.savefig(
    "figure6_gazebo_tex.png",
    dpi=300,
    bbox_inches="tight",
)

plt.close(fig)

print("\n=== FIGURE 6 GENERATED ===")
print("PDF:", output_pdf.resolve())
print("PNG:", Path("figure6_gazebo_tex.png").resolve())

print("\n=== OVERSHOOT RESULTS ===")

for controller, values in results.items():
    print(
        f"{controller}: "
        f"{values['mean']:.3f} +/- "
        f"{values['std']:.3f} px"
    )

print("\nDone.")
