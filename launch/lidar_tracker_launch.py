from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    bag_in_arg = DeclareLaunchArgument(
        'bag_in',
        description='Input bag file to play'
    )

    bag_out_arg = DeclareLaunchArgument(
        'bag_out',
        description='Output bag file to record'
    )

    bag_in = LaunchConfiguration('bag_in')
    bag_out = LaunchConfiguration('bag_out')

    # Record all topics (including outputs)
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_out],
        output='screen'
    )

    # Node 1: Scan processor
    scan_processor = Node(
        package='lidar_people_tracker',
        executable='scan_processor',
        name='scan_processor',
        output='screen'
    )

    # Node 2: Tracker / Marker Publisher
    tracker = Node(
        package='lidar_people_tracker',
        executable='tracker_node',
        name='tracker',
        output='screen'
    )

    # Play the input bag
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_in],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        bag_in_arg,
        bag_out_arg,
        play_bag,
        record_bag,
        scan_processor,
        tracker,
    ])
