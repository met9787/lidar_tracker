from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare bag input argument
    bag_in_arg = DeclareLaunchArgument(
        'bag_in',
        description='Input bag file to play'
    )

    bag_in = LaunchConfiguration('bag_in')

    # Nodes
    scan_processor = Node(
        package='lidar_tracker',
        executable='scan_processor',
        name='scan_processor',
        output='screen',
        emulate_tty=True
    )

    tracker = Node(
        package='lidar_tracker',
        executable='tracker_node',
        name='tracker_node',
        output='screen',
        emulate_tty=True
    )

    # RViz
    # rviz2 = ExecuteProcess(
    #     cmd=['ros2', 'run', 'rviz2', 'rviz2'],
    #     output='screen'
    # )

    # Play bag
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_in],
        output='screen'
    )

    return LaunchDescription([
        bag_in_arg,
        scan_processor,
        tracker,
        # rviz2,
        play_bag
    ])
