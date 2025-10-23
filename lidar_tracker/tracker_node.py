#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import math
import random


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker')

        self.subscription = self.create_subscription(
            PointStamped, '/people_positions', self.detection_callback, 10)
        self.publisher_ = self.create_publisher(MarkerArray, '/person_markers', 10)

        self.people = {}  # {id: {"history": [Point], "color": (r,g,b)}}
        self.next_id = 0
        self.distance_threshold = 0.8  # meters for matching detections

        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info("Tracker Node started. Publishing /person_markers")

    def detection_callback(self, msg: PointStamped):
        detected_point = msg.point

        # Find if this point belongs to an existing tracked person
        matched_id = None
        for pid, data in self.people.items():
            last_point = data["history"][-1]
            dist = math.sqrt(
                (detected_point.x - last_point.x) ** 2 +
                (detected_point.y - last_point.y) ** 2
            )
            if dist < self.distance_threshold:
                matched_id = pid
                break

        # If no match found, assign a new ID
        if matched_id is None:
            matched_id = self.next_id
            self.next_id += 1
            self.people[matched_id] = {
                "history": [],
                "color": (random.random(), random.random(), random.random())
            }

        # Append new position to history
        self.people[matched_id]["history"].append(detected_point)

        # Limit history length (optional)
        if len(self.people[matched_id]["history"]) > 100:
            self.people[matched_id]["history"].pop(0)

    def publish_markers(self):
        marker_array = MarkerArray()

        for pid, data in self.people.items():
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "people"
            marker.id = pid
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.05  # line width
            r, g, b = data["color"]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            marker.lifetime = Duration(sec=0)  # persistent
            marker.points = data["history"]

            marker_array.markers.append(marker)

        self.publisher_.publish(marker_array)
        self.get_logger().debug(f"Published {len(marker_array.markers)} markers")


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
