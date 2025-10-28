#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import numpy as np
from builtin_interfaces.msg import Time



class ScanProcessorNode(Node):
    def __init__(self):
        super().__init__('scan_processor')

        # Subscribe to LIDAR scans
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Publisher for detected "people positions"
        self.people_publisher = self.create_publisher(PointStamped, '/people_positions', 10)

        # Persistent environment model (initialized lazily)
        self.environment = None

        self.detection_log = []  # stores tuples of (timestamp, len(clusters))
        self.last_detected_count = None

        # Parameters for detection
        self.thresholds = [0.4]         # meters difference
        self.cluster_sizes = [10]         # consecutive beams to form valid cluster
        self.param_index = 0
        self.ignore_false = 3  # <-- number of consecutive 0's (False) to ignore and still merge clusters

        self.decay = 0.999  # slow environment update decay

        self.get_logger().info("✅ Scan Processor Node started (LIDAR change detection mode)")

    def scan_callback(self, msg: LaserScan):

        # --- Extract ranges and angles ---
        ranges = np.array(msg.ranges)
        num_points = len(ranges)
        angles = msg.angle_min + np.arange(num_points) * msg.angle_increment

        # Lazy initialization of environment model
        if self.environment is None:
            self.environment = ranges.copy()

        # --- Update environment model (keep max seen distances) ---
        self.environment = self.decay * self.environment + (1 - self.decay) * ranges
        self.environment = np.maximum(self.environment, ranges* 0.95)

        self.param_index = (self.param_index + 1) % (len(self.thresholds) * len(self.cluster_sizes))
        t_idx = self.param_index // len(self.cluster_sizes)
        c_idx = self.param_index % len(self.cluster_sizes)

        self.threshold = self.thresholds[t_idx]
        self.cluster_size = self.cluster_sizes[c_idx]

        # self.get_logger().info(f"Using threshold={self.threshold:.2f}, cluster_size={self.cluster_size}")


        # --- Compute difference from environment model ---
        delta = self.environment - ranges

        # Case 1: Normal closer-than-background detection
        poi_mask = (delta > self.threshold)

        # Case 2: Beam was previously invalid (inf/nan), now valid
        appeared_mask = np.isfinite(ranges) & ~np.isfinite(self.environment)

        # Combine both
        poi_mask = poi_mask | appeared_mask
        # self.get_logger().info(f"Detected {np.sum(poi_mask)} points of interest")
        
        self.get_logger().info(f"\n{poi_mask.astype(int)}")

        # --- Cluster consecutive detections ---
        clusters = []
        current_cluster = []
        gap_counter = 0
        self.min_cluster_size = self.cluster_size

        for i, is_poi in enumerate(poi_mask):
            if is_poi:
                # If current point is True, just add it
                current_cluster.append(i)
                gap_counter = 0  # reset the gap counter
            else:
                # Count gaps (zeros)
                if current_cluster:
                    gap_counter += 1
                    if gap_counter <= self.ignore_false:
                        # Still within tolerance, pretend the zeros don't break the cluster
                        continue
                    else:
                        # Too many zeros — end of cluster
                        if len(current_cluster) >= self.min_cluster_size:
                            clusters.append(current_cluster)
                        current_cluster = []
                        gap_counter = 0  # reset

        if self.last_detected_count != len(clusters):
            # Capture current ROS time in seconds (float)
            now = self.get_clock().now().seconds_nanoseconds()
            current_time = now[0] + now[1] * 1e-9

            # Initialize reference (start) time if this is the first entry
            if not hasattr(self, "start_time"):
                self.start_time = current_time

            # Compute time relative to start
            relative_time = current_time - self.start_time

            # Record time (1 decimal precision) and number of detected clusters
            self.detection_log.append((round(relative_time, 1), len(clusters), self.threshold, self.cluster_size))
            self.last_detected_count = len(clusters)

                # Don’t forget to check the last one at the end
        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(current_cluster)

            # Print in compact format
            print("Detection changes:")
            for idx, entry in enumerate(self.detection_log):
                print(f"{idx}: {entry}")
            print()



        # print("\n")
        # print(f"\rPerson(s) detected: {len(clusters)}\n\n")    

        # if len(clusters) > 0:
        #     # print("\n")
        #     # print(f"\rPerson(s) detected: {len(clusters)}\n\n")           
        #     self.get_logger().info(f"\nThreshold: {self.threshold} \n Cluster size: {self.cluster_size} \n POI: {len(poi_mask)} \n Detected {len(clusters)} clusters of interest\n\n")

        # --- Convert clusters to Cartesian & publish ---
        for cluster in clusters:
            cluster_ranges = ranges[cluster]
            cluster_angles = angles[cluster]

            # Compute Cartesian points for the entire cluster
            xs = cluster_ranges * np.cos(cluster_angles)
            ys = cluster_ranges * np.sin(cluster_angles)

            # If we have a previous point, pick the closest beam to it
            if hasattr(self, "last_point"):
                distances = np.sqrt((xs - self.last_point.x)**2 + (ys - self.last_point.y)**2)
                closest_idx = np.argmin(distances)
                x = xs[closest_idx]
                y = ys[closest_idx]
            else:
                # Default to cluster centroid for the very first detection
                x = np.mean(xs)
                y = np.mean(ys)

            # Publish
            point_msg = PointStamped()
            point_msg.header = Header()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = msg.header.frame_id
            point_msg.point = Point(x=float(x), y=float(y), z=0.0)

            self.people_publisher.publish(point_msg)

            # Save this as the last published point for continuity
            self.last_point = point_msg.point

        if clusters:
            self.get_logger().info(f"Published {len(clusters)} person positions.")


def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
