import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameNode(Node):
    """Plays a wide video and publishes only the crop the simulated camera sees."""

    def __init__(self):
        super().__init__('frame_node')
        self.declare_parameter('video_path', '/home/ubuntu/mini-SPIDR/data/wide.mp4')
        self.declare_parameter('crop_width', 640)
        self.declare_parameter('crop_height', 360)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('px_per_deg', 20.0)

        path = self.get_parameter('video_path').value
        self.crop_w = self.get_parameter('crop_width').value
        self.crop_h = self.get_parameter('crop_height').value
        fps = self.get_parameter('fps').value
        self.px_per_deg = self.get_parameter('px_per_deg').value

        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open video: {path}')
        self.wide_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.wide_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Video {self.wide_w}x{self.wide_h}, crop {self.crop_w}x{self.crop_h}')

        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, 'ptu/state', self.on_state, 10)
        self.create_timer(1.0 / fps, self.tick)

    def on_state(self, msg):
        self.pan_deg = msg.vector.x
        self.tilt_deg = msg.vector.y

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        cx = self.wide_w / 2 + self.pan_deg * self.px_per_deg
        cy = self.wide_h / 2 - self.tilt_deg * self.px_per_deg
        x0 = int(min(max(cx - self.crop_w / 2, 0), self.wide_w - self.crop_w))
        y0 = int(min(max(cy - self.crop_h / 2, 0), self.wide_h - self.crop_h))
        crop = frame[y0:y0 + self.crop_h, x0:x0 + self.crop_w]

        msg = self.bridge.cv2_to_imgmsg(crop, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FrameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
