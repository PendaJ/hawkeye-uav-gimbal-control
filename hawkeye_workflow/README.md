# Hawkeye consolidated workflow (Gazebo Harmonic + ROS 2 Humble)

A **practical launcher for the user's already tested Hawkeye X3 setup**. This launcher
has **not** been run on the user's Ubuntu workstation; the code has local syntax/
structure checks only. No existing ROS package, controller, X3 model, world, or
NPSO source is replaced.

## Installation

1. Stop manually launched Gazebo, ROS bridges, gimbal adapter, YOLO, and any SS-PD controller once. The launcher refuses to start a duplicate Gazebo world or duplicate adapter/tracker.
2. Extract `hawkeye_workflow.zip` **into `~/hawkeye_x3_ws`** (it contains the `hawkeye_workflow` directory).
3. `cd ~/hawkeye_x3_ws/hawkeye_workflow && chmod +x hawkeye`

Dependencies already known to work on the host:
- `/opt/ros/humble/setup.bash`, `~/hawkeye_x3_ws/install/setup.bash`
- `~/hawkeye_x3_ws/hawkeye_kinematic_carrier/{prepare_world.py,build/libHawkeyeCarrierMotion.so}`
- `~/hawkeye_x3_ws/.venv_yolo/bin/python`, installed `hawkeye_x3_perception`
- `~/Downloads/hawkeye_gazebo_sspd_controller_safe.py`
- optionally `~/hawkeye_x3_ws/rosbags/qos_override.yaml`
- optimization: `~/hawkeye_x3_ws/hawkeye_npso_gazebo/npso_gazebo.py`

## Normal workflow: **three commands**

```bash
cd ~/hawkeye_x3_ws/hawkeye_workflow
./hawkeye start lateral             # starts Gazebo GUI, camera and gimbal bridges, adapter, YOLO in the background
./hawkeye trial sspd 60             # record, run safe controller, stop, analyze automatically
./hawkeye stop                      # stops only processes created by the launcher
```

Other useful commands:

```bash
./hawkeye status
./hawkeye trial off 60              # no controller: uncompensated target error
./hawkeye trial zn_sspd 60          # ZN gain values in SS-PD realization (NOT classic ZN-PD)
./hawkeye trial custom 60 --kp 10 --kd 0.15 --wo 600
./hawkeye analyze                   # re-analyze last run
./hawkeye analyze ~/hawkeye_x3_ws/rosbags/lateral_sspd_20261104_143136 # existing manual bag
```

Runs appear in `~/hawkeye_x3_ws/rosbags/hawkeye_workflow/` with `bag/`,
`controller.log`, `run.json`, and `analysis/{summary.json,tracking.csv,commands.csv}`.
Console prints metrics automatically after each completed run.

## Existing offline NPSO optimization

**Use hold world for optimizer evaluation, not moving lateral world**:

```bash
./hawkeye stop
./hawkeye start hold
./hawkeye optimize selftest
./hawkeye optimize smoke
./hawkeye optimize pilot --evals 10 --seed 2026
./hawkeye optimize optimize --particles 20 --rounds 11 --seed 42
./hawkeye stop
```

The optimizer performs offline parameter search with **online Gazebo fitness
trials**; it is not an offline-only replay of a recorded dataset. The optimized
`best_parameters.json` can be used in `./hawkeye trial custom ...` after
restarting `lateral` or `combined` world for validation. Do not run NPSO and
trial commands simultaneously because they both publish `/cam_vel`.

## IMPORTANT study limitations / next automation step

- **Not yet publication-quality comparative runs:** prescribed motion is
  `y(t)=0.35 sin(0.5(t-3))` after 3 s hold, driven by Gazebo simulation time.
  Restarting and launching a controller at arbitrary wall-clock times does not
  synchronize the phase of the UAV trajectory at the start of a trial.
- Joint angles are **not reset automatically**. Fixed-gimbal and controller
  comparisons require identical initial yaw/pitch and target pose. Add a
  deterministic trial-start trigger and joint homing for fair repeated trials.
- `zn_sspd` applies Ziegler–Nichols Kp/Kd to the *same observer-based SS-PD
  implementation*; it is **not** the independent classical ZN-PD comparator.
- Analysis uses first and last nonzero velocity commands to approximate the
  active interval. In long zero-output periods, the inferred interval may
  underestimate the true timed window; inspect controller.log.
- Volatility `std(diff(r))` and mean second absolute difference of radial error
  are **per YOLO sample**, so their magnitudes depend on detector sampling
  frequency. Do not compare across runs with different sample rates without
  normalization/resampling.
- The NPSO gains came from the earlier Isaac Sim manuscript. Verify the Gazebo
  camera intrinsics / sign conventions before interpreting cross-platform
  performance. `start` checks that messages exist but does not verify sensor
  calibration or visual centering.
- Only the launcher-managed process groups are killed by `./hawkeye stop`.
  If `start` detects old terminals, close those manually; don't use `pkill -f`.
