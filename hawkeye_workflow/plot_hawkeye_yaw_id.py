#!/usr/bin/env python3
"""Plot the latest Hawkeye open-loop yaw identification experiment."""

import csv
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

DATA = Path.home() / "hawkeye_x3_ws/rosbags/hawkeye_workflow"

if len(sys.argv) > 1:
    run = Path(sys.argv[1]).expanduser()
else:
    choices = sorted(DATA.glob("yaw_identification_*/yaw_response.csv"))
    if not choices:
        raise SystemExit("No yaw identification results found.")
    run = choices[-1]

if run.is_dir():
    run = run / "yaw_response.csv"

with run.open(newline="") as f:
    rows = list(csv.DictReader(f))

if not rows:
    raise SystemExit("No response rows in " + str(run))

def series(source, key):
    selected = [r for r in rows if r["source"] == source]
    return (
        np.array([float(r["time_s"]) for r in selected]),
        np.array([float(r[key]) for r in selected])
    )

t, u = series("command", "command_yaw_rad_s")
tf, y = series("feedback", "yaw_deg")

if not len(t) or not len(tf):
    raise SystemExit("Missing command or feedback samples.")

y = y - y[0]

fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

axes[0].step(t, u, where="post", label="Yaw-rate command")
axes[0].set_ylabel("Yaw command (rad/s)")
axes[0].grid(True)
axes[0].legend()

axes[1].plot(tf, y, label="Measured yaw displacement")
axes[1].set_ylabel("Yaw displacement (deg)")
axes[1].set_xlabel("Time since test start (s)")
axes[1].grid(True)
axes[1].legend()

fig.suptitle("Hawkeye: Open-loop yaw identification")
fig.tight_layout()

output = run.parent / "yaw_identification.png"
fig.savefig(output, dpi=200)

print("Source:", run)
print("Saved plot:", output)
print("Feedback samples:", len(tf))
print("Command samples:", len(t))
print("Yaw displacement range:", np.min(y), np.max(y))
print("Yaw command range:", np.min(u), np.max(u))

plt.show()
