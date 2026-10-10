import rclpy
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class PID:
    """One-axis PID. Input: error in degrees. Output: rate command in deg/s."""

    def __init__(self, kp, ki, kd, out_limit, i_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        i_term = self.ki * self.integral

        if self.prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = p_term + i_term + d_term
        return max(-self.out_limit, min(self.out_limit, output))


class ControllerNode(Node):
    """Turns the detected target's pixel error into pan/tilt rate commands."""

    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('out_limit', 40.0)
        self.declare_parameter('i_limit', 10.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 360)
        self.declare_parameter('px_per_deg', 20.0)

        p = self.get_parameter
        self.img_w = p('image_width').value
        self.img_h = p('image_height').value
        self.px_per_deg = p('px_per_deg').value
        self.pan_pid = PID(p('kp').value, p('ki').value, p('kd').value,
                           p('out_limit').value, p('i_limit').value)
        self.tilt_pid = PID(p('kp').value, p('ki').value, p('kd').value,
                            p('out_limit').value, p('i_limit').value)
        self.last_t = None

        self.add_on_set_parameters_callback(self.on_params)
        self.create_subscription(Detection2DArray, 'detector/target', self.on_target, 10)
        self.pub_cmd = self.create_publisher(Vector3Stamped, 'cmd/auto', 10)
        self.pub_err = self.create_publisher(Vector3Stamped, 'controller/error_deg', 10)

    def on_params(self, params):
        for param in params:
            for pid in (self.pan_pid, self.tilt_pid):
                if param.name in ('kp', 'ki', 'kd', 'out_limit', 'i_limit'):
                    setattr(pid, param.name, param.value)
        return SetParametersResult(successful=True)

    def publish_cmd(self, stamp, pan_rate, tilt_rate):
        cmd = Vector3Stamped()
        cmd.header.stamp = stamp
        cmd.vector.x = pan_rate
        cmd.vector.y = tilt_rate
        self.pub_cmd.publish(cmd)

    def on_target(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = None if self.last_t is None else t - self.last_t
        self.last_t = t

        if not msg.detections or dt is None or dt <= 0.0:
            self.publish_cmd(msg.header.stamp, 0.0, 0.0)
            return

        det = msg.detections[0]
        err_x_px = det.bbox.center.position.x - self.img_w / 2
        err_y_px = det.bbox.center.position.y - self.img_h / 2

        err_pan = err_x_px / self.px_per_deg
        err_tilt = -err_y_px / self.px_per_deg

        err_msg = Vector3Stamped()
        err_msg.header = msg.header
        err_msg.vector.x = err_pan
        err_msg.vector.y = err_tilt
        self.pub_err.publish(err_msg)

        pan_rate = self.pan_pid.update(err_pan, dt)
        tilt_rate = self.tilt_pid.update(err_tilt, dt)

        self.publish_cmd(msg.header.stamp, pan_rate, tilt_rate)


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
