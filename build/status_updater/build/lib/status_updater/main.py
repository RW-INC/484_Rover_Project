import rclpy
import os
import subprocess
from rclpy.node import Node
from std_msgs.msg import Bool


class StatusUpdaterNode(Node):
    def __init__(self):
        super().__init__('status_updater_node')
        self.sparxe_pub = self.create_publisher(Bool, '/sparxe_status', 10)
        self.camera_pub = self.create_publisher(Bool, '/camera_status', 10)
        self.arduino_pub = self.create_publisher(Bool, '/arduino_status', 10)
        self.sparxe_pub_timer = self.create_timer(1.0, self.publish_sparxe_status)

    # Always publishes true since if this function is running it must mean that SPARX launch was successful
    def publish_sparxe_status(self):
        msg = Bool()
        msg.data = True
        self.sparxe_pub.publish(msg)
        self.publish_camera_status()
        self.publish_arduino_status()
        

    # Checks if the camera is connected and accessible, then publishes the status
    def publish_camera_status(self):
        device = "/dev/video0"
        if not os.path.exists(device):
            status = False
        else:
            # Checks if the camera is accessible
            try:
                result = subprocess.run(['v4l2-ctl', '--device', device, '--all'],
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL)
                if result.returncode == 0:
                    status = True
                else:
                    status = False
            except Exception as e:
                status = False
        msg = Bool()
        msg.data = status
        self.camera_pub.publish(msg)

    # Checks to see if path to the arduino exists, if it does assume connection and publish status
    def publish_arduino_status(self):
        device = "/dev/ttyACM0"
        if os.path.exists(device):
            status = True
        else:
            status = False
        msg = Bool()
        msg.data = status
        self.arduino_pub.publish(msg)

# Responsible for spinning up the StatusUpdaterNode and shutting it down when finished
def main():
    rclpy.init()
    node = StatusUpdaterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()