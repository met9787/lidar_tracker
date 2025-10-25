import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/metor/ros2_ws/src/lidar_tracker/install/lidar_tracker'
