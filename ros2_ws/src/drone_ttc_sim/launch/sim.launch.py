from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    sim_share = FindPackageShare('drone_ttc_sim')
    ros_gz_share = FindPackageShare('ros_gz_sim')

    default_world = PathJoinSubstitution([
        sim_share,
        'worlds',
        'drone_ttc.world',
    ])

    models_path = PathJoinSubstitution([
        sim_share,
        'models',
    ])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file to launch.',
    )

    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            models_path,
            ':',
            EnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                default_value='',
            ),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ros_gz_share,
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'gz_args': [
                '-r -v 3 ',
                LaunchConfiguration('world'),
            ],
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='drone_ttc_bridge',
        output='screen',
        arguments=[
            # Gazebo -> ROS
            '/drone/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/x3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # ROS -> Gazebo
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
    )

    return LaunchDescription([
        world_arg,
        resource_path,
        gazebo,
        bridge,
    ])
