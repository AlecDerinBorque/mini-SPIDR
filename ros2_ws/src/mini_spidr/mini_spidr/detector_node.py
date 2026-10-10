import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectorNode(Node):
    """Runs YOLOv8n on each camera frame and publishes the one target we track."""

    def __init__(self):
        super().__init__('detector_node')
        self.declare_parameter('model_path', '/home/ubuntu/mini-SPIDR/models/yolov8n.pt')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('gate_px', 150.0)
        self.declare_parameter('forget_after', 10)

        self.model = YOLO(self.get_parameter('model_path').value)
        target = self.get_parameter('target_class').value
        names = self.model.names
        ids = [i for i, n in names.items() if n == target]
        if not ids:
            raise RuntimeError(f'Unknown class "{target}". Valid: {sorted(names.values())}')
        self.class_id = ids[0]
        self.target_name = target
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.gate_px = self.get_parameter('gate_px').value
        self.forget_after = self.get_parameter('forget_after').value

        self.bridge = CvBridge()
        self.last_center = None
        self.misses = 0
        self.infer_ms = []
        self.last_log = time.time()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos)
        self.pub = self.create_publisher(Detection2DArray, 'detector/target', 10)
        self.pub_debug = self.create_publisher(Image, 'detector/debug_image', qos)
        self.get_logger().info(f'Tracking class "{target}" (id {self.class_id})')

    def pick_target(self, candidates):
        if not candidates:
            return None
        if self.last_center is not None:
            lx, ly = self.last_center
            nearest = min(candidates, key=lambda c: (c[0] - lx) ** 2 + (c[1] - ly) ** 2)
            dist = ((nearest[0] - lx) ** 2 + (nearest[1] - ly) ** 2) ** 0.5
            if dist <= self.gate_px:
                return nearest
        return max(candidates, key=lambda c: c[4])

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        t0 = time.perf_counter()
        result = self.model.predict(
            img, imgsz=self.imgsz, conf=self.conf, classes=[self.class_id],
            device='cpu', verbose=False,
        )[0]
        self.infer_ms.append(1000 * (time.perf_counter() - t0))

        candidates = []
        for box in result.boxes:
            cx, cy, w, h = box.xywh[0].tolist()
            candidates.append((cx, cy, w, h, float(box.conf[0])))

        target = self.pick_target(candidates)

        out = Detection2DArray()
        out.header = msg.header
        if target is not None:
            cx, cy, w, h, score = target
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.target_name
            hyp.hypothesis.score = score
            det.results.append(hyp)
            out.detections.append(det)
            self.last_center = (cx, cy)
            self.misses = 0
        else:
            self.misses += 1
            if self.misses >= self.forget_after:
                self.last_center = None
        self.pub.publish(out)

        if self.pub_debug.get_subscription_count() > 0:
            self.publish_debug(img, candidates, target, msg.header)

        now = time.time()
        if now - self.last_log > 2.0 and self.infer_ms:
            avg = sum(self.infer_ms) / len(self.infer_ms)
            self.get_logger().info(
                f'inference {avg:.0f} ms (~{1000 / avg:.1f} FPS), {len(candidates)} candidates')
            self.infer_ms.clear()
            self.last_log = now

    def publish_debug(self, img, candidates, target, header):
        h, w = img.shape[:2]
        cv2.drawMarker(img, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
        for c in candidates:
            x0, y0 = int(c[0] - c[2] / 2), int(c[1] - c[3] / 2)
            x1, y1 = int(c[0] + c[2] / 2), int(c[1] + c[3] / 2)
            cv2.rectangle(img, (x0, y0), (x1, y1), (160, 160, 160), 1)
        if target is not None:
            cx, cy, bw, bh, score = target
            x0, y0 = int(cx - bw / 2), int(cy - bh / 2)
            x1, y1 = int(cx + bw / 2), int(cy + bh / 2)
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(img, f'{self.target_name} {score:.2f}', (x0, max(y0 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(img, 'NO TARGET', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        dbg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        dbg.header = header
        self.pub_debug.publish(dbg)


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
