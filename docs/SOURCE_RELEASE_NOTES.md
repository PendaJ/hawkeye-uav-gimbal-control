# Source release notes

This package was curated from `hawkeye_project_source.tar.gz` supplied for the Hawkeye research repository.

## Included
- ROS 2 Python packages for camera tracking and velocity-command gimbal adaptation
- Gazebo Harmonic yaw–pitch gimbal and UAV worlds/models
- Workflow scripts for launching, baseline trials, plotting, and analysis

## Not included
- Historical `.before_*`, `.bak`, and swap-file copies
- Unrelated TTC controller package and TTC-only experiment world
- ROS 2 build outputs, bag recordings, detector weights, and experiment data
- NPSO implementation (`hawkeye_npso_gazebo/npso_gazebo.py`, `npso_trial.py`): **not present in supplied archive**
- SS-PD controller under `~/Downloads/hawkeye_gazebo_sspd_controller_safe.py`: **not present in supplied archive**
- QoS override and normalization files referenced by scripts: **not present in supplied archive**

This upload is incomplete for running Experiments 2–7 end-to-end; add missing dependencies before claiming fully reproducible results.

The gimbal interface consumes `/cam_vel` velocity commands (`geometry_msgs/Twist`), with `angular.x` for pitch and `angular.y` for yaw.

The perception README refers to `yolo26n.pt`; actual detector weights are omitted and license must be checked. Gazebo SDF models reference external Fuel assets and may require downloading those.
