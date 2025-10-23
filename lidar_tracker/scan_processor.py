#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32, PointStamped
from std_msgs.msg import Header
import numpy as np
from math import sin, cos


class ScanProcessorNode(Node):
    def __init__(self):
        super().__init__('scan_processor')

        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.cloud_pub = self.create_publisher(PointCloud, '/scan_cloud', 10)
        self.people_pub = self.create_publisher(PointStamped, '/people_positions', 10)

        self.get_logger().info("✅ Scan Processor Node started. Subscribed to /scan")
    
        self.count = 0
    # ----------------------------------------------------------------------
    # ✅ Converts LaserScan → PointCloud
    # ----------------------------------------------------------------------
    # def laser_scan_to_point_cloud(self, scan: LaserScan) -> PointCloud:
    #     """
    #     Converts a LaserScan message to a PointCloud message.
    #     Properly references scan.header, scan.angle_min, etc.
    #     """
    #     cloud = PointCloud()
    #     cloud.header = Header()
    #     cloud.header.stamp = scan.header.stamp
    #     # Make sure to use the LIDAR's frame or remap if needed
    #     cloud.header.frame_id = scan.header.frame_id or "laser"

    #     points = []
    #     for i in range(len(scan.ranges)):
    #         r = scan.ranges[i]
    #         # skip invalid or out-of-range readings
    #         if np.isfinite(r) and scan.range_min < r < scan.range_max:
    #             angle = scan.angle_min + i * scan.angle_increment
    #             x = r * cos(angle)
    #             y = r * sin(angle)
    #             points.append(Point32(x=x, y=y, z=0.0))

    #     cloud.points = points
    #     return cloud
    
    def laser_scan_to_point_cloud(self, scan: LaserScan):
        # self.count += 1
        # self.get_logger().info(f"Converting scan #{self.count} to PointCloud with {len(scan.ranges)} points")
        return PointCloud(header=scan.header, points=[
            Point32(x=scan.ranges[i] * cos(scan.angle_min + i * scan.angle_increment),
                     y=scan.ranges[i] * sin(scan.angle_min + i * scan.angle_increment),
                     z=0.0)
            for i in range(len(scan.ranges))
        ])

    # ----------------------------------------------------------------------
    # ✅ Callback for /scan messages
    # ----------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        # Convert to PointCloud and publish
        cloud = self.laser_scan_to_point_cloud(msg)
        self.cloud_pub.publish(cloud)

        # If empty, skip
        if len(cloud.points) == 0:
            self.get_logger().warn("⚠️ No valid scan points found.")
            return

        # Convert to numpy arrays
        xs = np.array([p.x for p in cloud.points])
        ys = np.array([p.y for p in cloud.points])

        # (For now, randomly select mock “people”)
        num_people = np.random.randint(1, 3)
        indices = np.random.choice(len(xs), num_people, replace=False)

        for idx in indices:
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = msg.header.frame_id or "base_link"
            pt.point.x = float(xs[idx])
            pt.point.y = float(ys[idx])
            pt.point.z = 0.0
            self.people_pub.publish(pt)

        self.get_logger().debug(f"Published {num_people} mock people positions")


# ----------------------------------------------------------------------
# ✅ Standard ROS2 boilerplate
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Scan Processor.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
