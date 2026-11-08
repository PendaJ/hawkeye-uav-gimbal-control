# Source release audit — November 8, 2026

The ROS 2 / Gazebo source archive provided for the Hawkeye project contains:
- `hawkeye_x3_gimbal` ROS 2 package with a `/cam_vel` yaw–pitch rate adapter;
- `hawkeye_x3_perception` target tracker;
- `drone_ttc_sim` Gazebo worlds and UAV/gimbal models;
- `hawkeye_workflow` launch, baseline-control, trial, plotting, and analysis scripts.

**Not yet uploaded to this repository:** The complete ROS 2 source package files are packaged for review and local Git push.

**Missing from the source archive:** `hawkeye_npso_gazebo/npso_gazebo.py`, `hawkeye_npso_gazebo/npso_trial.py`, `~/Downloads/hawkeye_gazebo_sspd_controller_safe.py`, normalization reference JSON, and the referenced ROS 2 QoS override.

The control interface is velocity-based: `geometry_msgs/msg/Twist` on `/cam_vel`, with `angular.x` for pitch and `angular.y` for yaw. Do not describe this as an Isaac Sim position-command interface.

No secrets matching common token/private-key patterns were found in the curated source files, but the repository owner should review all files before public upload. Unlicensed detector weights, backups, ROS bags, and generated build outputs are excluded.

**Reproducibility status: incomplete** until source dependencies, documented model weights, and evaluation data are supplied, and build instructions are validated.
