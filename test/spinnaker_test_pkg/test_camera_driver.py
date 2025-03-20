import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pytest

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
        global received
        received[cam] = True
        self.get_logger().info(f'✅ Camera image received from {cam}!')

@pytest.mark.ros2
def test_image_stream_from_both_cameras():
    global received
    rclpy.init()
    node = ImageTestNode()
    try:
        timeout = 15
        end_time = node.get_clock().now().nanoseconds / 1e9 + timeout
        while not all(received.values()) and node.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    assert received['cam0'], "❌ No image received from cam0"
    assert received['cam1'], "❌ No image received from cam1"
