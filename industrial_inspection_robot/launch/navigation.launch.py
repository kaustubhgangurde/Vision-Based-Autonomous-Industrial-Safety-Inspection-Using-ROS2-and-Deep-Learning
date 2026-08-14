import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'industrial_inspection_robot'
    pkg_share = get_package_share_directory(package_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Master Simulation Launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'sim.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. SLAM Toolbox Online Async Mapping Node
    slam_config = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}]
    )

    # 3. Vision Safety Hazard Detector Node
    hazard_detector_node = Node(
        package=package_name,
        executable='hazard_detector.py',
        name='hazard_detector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. Autonomous Inspection Route Manager Node
    inspection_manager_node = Node(
        package=package_name,
        executable='inspection_manager.py',
        name='inspection_manager_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. Automated Inspection Report Generator Node
    report_generator_node = Node(
        package=package_name,
        executable='report_generator.py',
        name='report_generator_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 6. RViz2 Visualization Node
    rviz_config = os.path.join(pkg_share, 'config', 'inspection.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        sim_launch,
        slam_node,
        hazard_detector_node,
        inspection_manager_node,
        report_generator_node,
        rviz_node
    ])
