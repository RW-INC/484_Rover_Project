#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import csv
import math
import threading
import time
import os

# NOTE: this class structure was pretty much ripped off of the teleop_twist_keyboard source code
# creating custom thread class
class PublishThread(threading.Thread):
    def __init__(self, node, rate):
        super(PublishThread, self).__init__() # parent thread
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10) # making publisher for cmd_vel
        self.x = 0.0    # linear velocity
        self.th = 0.0   # angular velocity
        self.condition = threading.Condition() # helps share info between threads
        self.done = False # adding a termination flag for the thread
        self.timeout = 1.0 / rate if rate != 0.0 else None # 20 Hz datarate
        self.node = node # save node and start thread immediately
        self.start()

    # function for velocity updates
    def update(self, x, th):
        self.condition.acquire()
        self.x = x
        self.th = th
        self.condition.notify()
        self.condition.release()

    # safely stop thread
    def stop(self):
        self.done = True
        self.update(0.0, 0.0)
        self.join()

    # thread loop
    def run(self):
        while not self.done: # loop forever until termination condition

            # wait for new data or until timeout
            self.condition.acquire()
            self.condition.wait(self.timeout)

            # copying the state into a twist msg for velocity
            twist = Twist()
            twist.linear.x = float(self.x)
            twist.angular.z = float(self.th)
            self.condition.release()

            # publish velocity to rover
            if rclpy.ok():
                self.publisher.publish(twist)

        # publish stop when exiting
        self.publisher.publish(Twist())

# main control class
class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # configuration 
        self.pixel_scale = 1.0         # 10m per pixel scale
        self.img_center_x = 860.0       # change in accordance to tiff size
        self.img_center_y = 585.5       # change in accordance to tiff size
        self.waypoint_threshold = 5.0   # if within 2 meters of a waypoint validate completion 

        # load path from csv
        # add new paths in folders with labeled path length
        package_share = get_package_share_directory('rover_sim')
        csv_path = os.path.join(package_share, 'scripts', '6211m', 'path_6221_PIXELS.csv')
        self.waypoints = self.load_path_from_csv(csv_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")

        self.current_idx = 0        # current waypoint naving to
        self.current_pose = None    # current position of rover

        self.pub_thread = PublishThread(self, 20.0) # 20 Hz
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # subscribe to odom to listen to position updates

    # function to extract path from csv file
    # csv is from Sayali code
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
            self.get_logger().error(f"csv error D: {e}")

        return world_points

    # calculating heading angle from orientation information
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp) # return direction in rads

    # odometry callback, runs every position update
    def odom_callback(self, msg):
        # this callback MUST fire to update 'x' and 'th'
        self.current_pose = msg.pose.pose # save position

        # check if all waypoints have been reached
        if self.current_idx >= len(self.waypoints):
            self.pub_thread.update(0.0, 0.0) # if so, stop

            return

        # getting target and current position
        target_x, target_y = self.waypoints[self.current_idx]
        curr_x, curr_y = self.current_pose.position.x, self.current_pose.position.y

        # calc distance to next waypoint
        dist = math.hypot(target_x - curr_x, target_y - curr_y)
        
        # stop moving if you're close enough to the waypoint
        if dist < self.waypoint_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_idx}") # send confirmation to terminal
            self.current_idx += 1
            self.pub_thread.update(0.0, 0.0)
        #if not continue to nav towards it
        else:
            desired_yaw = math.atan2(target_y - curr_y, target_x - curr_x)
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation) # feed current direction into yaw function
            # and calculate difference in current heading vs desired heading
            angle_error = math.atan2(math.sin(desired_yaw - current_yaw), 
                                     math.cos(desired_yaw - current_yaw))
            
            # send new values to the PublishThread
            # you can change max speed by changing the min functin params here
            # current max speed = 1 m/s
            self.pub_thread.update(min(1, 0.2 * dist), 1.0 * angle_error)

def main():
    rclpy.init()
    node = PathFollower()
    
    # required for multithreading support in ROS2
    # allows the thread and callbacks to run simultaneously 
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # clean shutdown 
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_thread.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
