import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Creates the node that starts the camera stream
class StreamCameraNode(Node):
    def __init__(self):    
        super().__init__('stream_camera_node')

        self.laptop_ip = None
        self.last_ready_time = 0.0
        self.timeout = 3.0
        self.process = None

        # Subscription that listens for laptop ip
        self.sub = self.create_subscription(
            String,
            '/video_client_info',
            self.info_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.check_stream)

    # Callback for laptop ip subscription and saves time it was received
    def info_callback(self, msg):
        self.laptop_ip = msg.data
        self.last_ready_time = time.time()

    # If not laptop IP received, does nothing, otherwise starts the stream if not already running
    def start_stream(self):
        if not self.laptop_ip:
            return
        if self.process is None or self.process.poll() is not None:
            self.process = subprocess.Popen([
                'ffmpeg',
                '-f', 'v4l2',
                '-input_format', 'mjpeg',
                '-video_size', '640x480',
                '-framerate', '30',
                '-i', '/dev/video0',
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                '-tune', 'zerolatency',
                '-fflags', 'nobuffer',
                '-flags', 'low_delay',
                '-g', '30',
                '-pix_fmt', 'yuv420p',
                '-f', 'mpegts',
                f'udp://{self.laptop_ip}:1234'
            ])

    # If stream is already running but not update from laptop, stream is shut down
    def stop_stream(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            self.process = None
            self.get_logger().info('Stopped stream')

    # If to much time as passed since subscription received, it will shut down the stream
    def check_stream(self):
        ready_recently = (time.time() - self.last_ready_time) < self.timeout
        if ready_recently:
            self.start_stream()
        else:
            self.stop_stream()

# Responsible for spinning up the camera streaming node
def main():
    rclpy.init()
    node = StreamCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
