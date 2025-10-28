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
        super().__init__('tracker_node')

        # Subscribe to detections
        self.subscription = self.create_subscription(
            PointStamped, '/people_positions', self.detection_callback, 10)

        # Publisher for visualization markers
        self.publisher_ = self.create_publisher(MarkerArray, '/person_markers', 10)

        # Tracked people storage
        self.people = {}  # {id: {"history": [Point], "color": (r,g,b)}}
        self.next_id = 1
        self.distance_threshold = 0.8  # meters for matching detections

        # clear all published markers on startup
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.publisher_.publish(marker_array)

        self.get_logger().info("✅ Tracker Node started — listening to /people_positions")

    # ---------------------------------------------------------
    # Detection callback: receives PointStamped messages
    # ---------------------------------------------------------
    def detection_callback(self, msg: PointStamped):
        detected_point = msg.point

        # Capture current ROS time in seconds (float)
        now = self.get_clock().now().seconds_nanoseconds()
        current_time = now[0] + now[1] * 1e-9

        # Initialize start time (reference time) once
        if not hasattr(self, "start_time"):
            self.start_time = current_time

        # Compute elapsed time since start
        elapsed_time = current_time - self.start_time

        # Log detection info with timestamp (1 decimal precision)
        self.get_logger().info(
            f"[{elapsed_time:.1f}s] Received detection at ({detected_point.x:.2f}, {detected_point.y:.2f})"
        )

        r = math.sqrt(detected_point.x**2 + detected_point.y**2)
        self.distance_threshold = max(0.5, min(1.0, 0.3 + 0.125 * r))

        # Try to associate detection with existing tracked person
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

        # If no match found, assign a new person ID
        if matched_id is None:
            matched_id = self.next_id
            self.next_id += 1
            self.people[matched_id] = {
                "history": [],
                "color": (random.random(), random.random(), random.random()),
                "motion_line": None,
                "last_update_time": current_time,
                "active": True
            }

        # --- Spike filtering ---
        person_data = self.people[matched_id]
        history = person_data["history"]

        # Initialize short-term smoothing window if missing
        if "recent_window" not in person_data:
            person_data["recent_window"] = []

        # Apply delayed spike smoothing (maintains full history)
        smoothed_history, recent_window = self.smooth_with_future_check(
            detected_point, history, person_data["recent_window"]
        )
        person_data["history"] = smoothed_history
        person_data["recent_window"] = recent_window

        # --- Stationary false detection cleanup ---
        min_required_points = 80        # Only check after enough data
        min_movement_distance = 0.3     # Minimum travel distance (meters)
        to_remove = []



        for pid, data in self.people.items():
            hist = data["history"]
            if len(hist) >= min_required_points:
                first_p, last_p = hist[0], hist[-1]
                total_dist = math.sqrt(
                    (last_p.x - first_p.x) ** 2 + (last_p.y - first_p.y) ** 2
                )
                if total_dist < min_movement_distance:
                    self.get_logger().warn(
                        f"🟠 Removing stationary false track ID {pid}: "
                        f"only moved {total_dist:.2f} m over {len(hist)} detections."
                    )
                    to_remove.append(pid)

        for pid in to_remove:
            del self.people[pid]

        # Log the smoothed path
        formatted_path = [f"({p.x:.2f}, {p.y:.2f})" for p in smoothed_history]
        print(f"\nPerson {matched_id} smoothed path ({len(smoothed_history)} points): {formatted_path}")

        # Publish markers as usual
        self.publish_markers()
        self.get_logger().info(f"END OF CURRENT SCAN ANALYSIS \n \n ")




    
    # ---------------------------------------------------------
    # Marker publishing for visualization
    # ---------------------------------------------------------
    def publish_markers(self):
        marker_array = MarkerArray()

        for pid, data in self.people.items():
            marker = Marker()
            marker.header.frame_id = "laser" 
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tracked_people"
            marker.id = pid
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.08  # line width

            # Assign color per person
            r, g, b = data["color"]
            marker.color.r = float(r)
            marker.color.g = float(g)
            marker.color.b = float(b)
            marker.color.a = 1.0

            marker.lifetime = Duration(sec=0)  # persistent marker

            # Add all past points to the line
            for p in data["history"]:
                marker.points.append(Point(x=p.x, y=p.y, z=0.0))

            marker_array.markers.append(marker)

        # Publish to RViz
        self.publisher_.publish(marker_array)
        self.get_logger().info(f"Published {len(marker_array.markers)} person paths.")
        


    def filter_spike(self, new_point: Point, history: list, spike_threshold: float = 0.30) -> Point:
        """
        Detect and correct spikes in incoming point detections.
        Uses the past 5 points to estimate motion trend.
        If the new point deviates too far from the trend, it is replaced
        with an interpolated continuation of recent motion.
        """
        if len(history) < 5:
            # Not enough points yet to establish a trend
            return new_point

        # Use the last 5 points
        p_hist = history[-5:]  # p1...p5 (oldest to newest)

        # Compute average velocity vector from consecutive points
        dxs = [p_hist[i+1].x - p_hist[i].x for i in range(4)]
        dys = [p_hist[i+1].y - p_hist[i].y for i in range(4)]
        avg_dx = sum(dxs) / len(dxs)
        avg_dy = sum(dys) / len(dys)

        # Predict the next point assuming constant velocity
        last_point = p_hist[-1]
        predicted = Point(
            x=last_point.x + avg_dx,
            y=last_point.y + avg_dy,
            z=0.0
        )

        # Compute deviation from expected trend
        deviation = math.sqrt(
            (new_point.x - predicted.x) ** 2 + (new_point.y - predicted.y) ** 2
        )

        if deviation > spike_threshold:
            # Prepare debug info for before/after
            old_vals = [(p.x, p.y) for p in p_hist[-4:]] + [(new_point.x, new_point.y)]
            new_vals = [(p.x, p.y) for p in p_hist[-4:]] + [(predicted.x, predicted.y)]

            self.get_logger().warn(
                f"Spike detected (Δ={deviation:.2f}m). Replacing point."
                f"\n  Before: {[(f'({x:.2f},{y:.2f})') for x, y in old_vals]}"
                f"\n  After:  {[(f'({x:.2f},{y:.2f})') for x, y in new_vals]}"
            )
            return predicted
        else:
            return new_point

    def smooth_with_future_check(self, new_point: Point, full_history: list, recent_window: list, spike_threshold: float = 0.30):
        """
        Temporal smoothing method (delayed spike correction):
        - Keeps last 3 points in a short-term window.
        - Only commits the middle point once it’s validated.
        - Retroactively replaces it if it deviates too far from trend.
        - Maintains full smoothed history.
        """
        recent_window.append(new_point)

        # Keep only the last 3 points
        if len(recent_window) > 3:
            recent_window.pop(0)

        # Not enough points yet to decide (need 3)
        if len(recent_window) < 3:
            # For the first two points, just store them
            full_history.append(new_point)
            return full_history, recent_window

        # Now we have 3 points: p1, p2, p3
        p1, p2, p3 = recent_window

        # Predict expected midpoint (linear interpolation)
        interp_x = (p1.x + p3.x) / 2.0
        interp_y = (p1.y + p3.y) / 2.0
        deviation = math.sqrt((p2.x - interp_x) ** 2 + (p2.y - interp_y) ** 2)

        # Correct if needed
        if deviation > spike_threshold:
            self.get_logger().warn(
                f"Retroactive spike correction (Δ={deviation:.2f}m):"
                f"\n  Before: ({p1.x:.2f},{p1.y:.2f}) -> ({p2.x:.2f},{p2.y:.2f}) -> ({p3.x:.2f},{p3.y:.2f})"
                f"\n  After:  ({p1.x:.2f},{p1.y:.2f}) -> ({interp_x:.2f},{interp_y:.2f}) -> ({p3.x:.2f},{p3.y:.2f})"
            )
            p2 = Point(x=interp_x, y=interp_y, z=0.0)
            recent_window[1] = p2  # update in window

        # ✅ Append the confirmed (middle) point only
        full_history.append(p2)

        return full_history, recent_window




# ---------------------------------------------------------
# Entry point
# ---------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
