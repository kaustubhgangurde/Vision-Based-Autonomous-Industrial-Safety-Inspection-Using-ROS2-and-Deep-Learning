import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'industrial_inspection_robot'

    # Declare launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to the world file
    pkg_share = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share, 'config', 'burger_world.sdf')

    # Include Gazebo Simulation Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Robot State Publisher Node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn the Robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'industrial_inspection_robot',
            '-z', '0.1'
        ],
        output='screen'
    )

    # ROS 2 to Gazebo Parameter Bridge Configuration
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        gazebo,
        rsp,
        spawn_robot,
        bridge
    ])
