#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import csv
import math
import threading
import time
import os

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Configuration
        self.pixel_scale = 10.0
        self.img_center_x = 101.5
        self.img_center_y = 101.0
        self.waypoint_threshold = 2.0 

        # Load Path
        package_share = get_package_share_directory('rover_sim')
        csv_path = os.path.join(package_share, 'scripts', '30m', 'cropped_path_pixels_30m.csv')
        self.waypoints = self.load_path_from_csv(csv_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")

        self.current_idx = 0
        self.current_pose = None
        self.twist_msg = Twist()
        
        # Thread setup
        self.condition = threading.Condition()
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def load_path_from_csv(self, file_path):
        world_points = []
        try:
            with open(file_path, mode='r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    px, py = float(row['x']), float(row['y'])
                    gx = (px - self.img_center_x) * self.pixel_scale
                    gy = (self.img_center_y - py) * self.pixel_scale
                    world_points.append((gx, gy))
        except Exception as e:
            self.get_logger().error(f"CSV Load Error: {e}")
        return world_points

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        with self.condition:
            self.current_pose = msg.pose.pose

            if self.current_idx >= len(self.waypoints):
                self.twist_msg = Twist()
                return

            target_x, target_y = self.waypoints[self.current_idx]
            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y

            dist = math.hypot(target_x - curr_x, target_y - curr_y)
            
            if dist < self.waypoint_threshold:
                self.get_logger().info(f"Reached waypoint {self.current_idx}")
                self.current_idx += 1
                self.twist_msg = Twist()
            else:
                desired_yaw = math.atan2(target_y - curr_y, target_x - curr_x)
                current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
                angle_error = math.atan2(math.sin(desired_yaw - current_yaw), math.cos(desired_yaw - current_yaw))
                
                new_cmd = Twist()
                new_cmd.linear.x = min(0.5, 0.2 * dist)
                new_cmd.angular.z = 1.0 * angle_error
                self.twist_msg = new_cmd
            
            # Notify the publish thread that twist_msg is updated
            self.condition.notify()

    def publish_loop(self):
        # Wait for subscriber like the teleop reference
        while rclpy.ok() and self.cmd_pub.get_subscription_count() == 0:
            time.sleep(0.1)
        
        self.get_logger().info("Subscriber connected. Publishing...")
        
        rate = 20.0 # Hz
        while rclpy.ok():
            with self.condition:
                # We publish the latest twist_msg calculated in the odom_callback
                self.cmd_pub.publish(self.twist_msg)
            time.sleep(1.0 / rate)

def main():
    rclpy.init()
    node = PathFollower()
    
    # CRITICAL: Use MultiThreadedExecutor so the odom_callback 
    # isn't blocked by the while loop or the publish thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
