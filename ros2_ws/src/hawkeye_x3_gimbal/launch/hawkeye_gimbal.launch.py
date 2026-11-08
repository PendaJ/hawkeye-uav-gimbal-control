"""Run TTC X3 simulator with Hawkeye gimbal world, bridge, and safe adapter."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare('drone_ttc_sim')
    sim_launch = PathJoinSubstitution([sim_pkg, 'launch', 'sim.launch.py'])
    world = PathJoinSubstitution([sim_pkg, 'worlds', 'hawkeye_gimbal.world'])

    original_x3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'world': world}.items(),
    )

    gimbal_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='hawkeye_gimbal_bridge',
        output='screen',
        arguments=[
            '/model/x3/joint/gimbal_yaw_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/x3/joint/gimbal_pitch_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/x3/gimbal_joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
    )

    adapter = Node(
        package='hawkeye_x3_gimbal',
        executable='gimbal_adapter',
        name='hawkeye_gimbal_adapter',
        output='screen',
        parameters=[{
            'max_velocity_rad_s': 0.2,
            'command_timeout_s': 0.5,
        }],
    )
    return LaunchDescription([original_x3_sim, gimbal_bridge, adapter])
