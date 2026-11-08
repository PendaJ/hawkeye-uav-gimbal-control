# Hawkeye X3 gimbal ROS 2 adapter

Add this package into `~/hawkeye_x3_ws/src`, build and run:

```bash
source /opt/ros/humble/setup.bash
cd ~/hawkeye_x3_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch hawkeye_x3_gimbal hawkeye_gimbal.launch.py
```

Run this launch **instead of**, not alongside, `ros2 launch drone_ttc_sim sim.launch.py`.

### Interfaces

- `/cam_vel` (`geometry_msgs/msg/Twist`), `angular.x` pitch and `angular.y` yaw in rad/s
- `/current_orientation` (`geometry_msgs/msg/Vector3`), `x` pitch and `z` yaw in degrees
- `/model/x3/gimbal_joint_state` (`sensor_msgs/msg/JointState`), from the Gazebo bridge
- `/model/x3/joint/gimbal_yaw_joint/cmd_vel` and pitch equivalent (`std_msgs/msg/Float64`)

For initial tests, adapter clamps to +/-0.2 rad/s and returns both rates to zero if no command arrives for 0.5s. It does not implement position hold or gimbal stabilization and is not a safety-certified controller. Keep the SS-PD controller stopped while testing manual commands.
