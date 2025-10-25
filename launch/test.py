from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import launch.events

def generate_launch_description():
    # --- Declare input argument for bag file ---
    bag_in_arg = DeclareLaunchArgument(
        'bag_in',
        description='Input bag file to play'
    )

    bag_in = LaunchConfiguration('bag_in')

    # --- Nodes ---
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

    # --- ros2 bag play process ---
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_in],
        output='screen'
    )

    terminate_at_end = RegisterEventHandler(
        OnProcessExit(
            target_action=play_bag,
            on_exit=[
                EmitEvent(event=launch.events.Shutdown())

            ]
        )
    )

    # --- Return complete launch description ---
    return LaunchDescription([
        bag_in_arg,
        scan_processor,
        tracker,
        play_bag,
        terminate_at_end
    ])
