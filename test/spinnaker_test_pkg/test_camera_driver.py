import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pytest

# Check if hardware is connected via an environment variable (e.g., "HARDWARE_CONNECTED")
hardware_connected = os.getenv('HARDWARE_CONNECTED', 'false').lower() == 'true'

# Dictionary to track whether an image is received from each camera
received = {
    'cam0': False,
    'cam1': False
}

class ImageTestNode(Node):
    def __init__(self):
        super().__init__('pilot02_physical_blackfly_camera_tester')
        self.sub_list = []
        for cam in ['cam0', 'cam1']:
            topic = f'/cam_sync/{cam}/image_raw'
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, cam=cam: self.listener_callback(msg, cam),
                10
            )
            self.sub_list.append(sub)

    def listener_callback(self, msg, cam):
        # Update the received dictionary when a message is received
        global received
        received[cam] = True
        self.get_logger().info(f'✅ Camera image received from {cam}!')

def test_image_stream_from_both_cameras():
    global received

    print(f"Hardware connected: {hardware_connected}")
    if not hardware_connected:
        # If no hardware is connected, skip the test but do not mark it as a failure
        print("⚠️ No hardware detected. Skipping hardware-dependent test.")
        return  # Skip the test and implicitly mark it as passed.

    # If hardware is connected, run the test normally
    rclpy.init()
    node = ImageTestNode()
    try:
        timeout = 15  # Time in seconds
        end_time = node.get_clock().now().nanoseconds / 1e9 + timeout
        while not all(received.values()) and node.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Assert that both cameras have received an image
    assert received['cam0'], "❌ No image received from cam0"
    assert received['cam1'], "❌ No image received from cam1"
