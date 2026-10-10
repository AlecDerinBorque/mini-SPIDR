import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


def clamp(value, low, high):
    return max(low, min(high, value))


class PtuSimNode(Node):
    """Simulated pan/tilt unit: integrates rate commands into angles, with limits."""

    def __init__(self):
        super().__init__('ptu_sim_node')
        self.declare_parameter('max_rate', 40.0)
        self.declare_parameter('max_accel', 300.0)
        self.declare_parameter('pan_min', -16.0)
        self.declare_parameter('pan_max', 16.0)
        self.declare_parameter('tilt_min', -9.0)
        self.declare_parameter('tilt_max', 9.0)
        self.declare_parameter('update_hz', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.max_rate = self.get_parameter('max_rate').value
        self.max_accel = self.get_parameter('max_accel').value
        self.pan_lim = (self.get_parameter('pan_min').value, self.get_parameter('pan_max').value)
        self.tilt_lim = (self.get_parameter('tilt_min').value, self.get_parameter('tilt_max').value)
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.pan = 0.0
        self.tilt = 0.0
        self.vel_pan = 0.0
        self.vel_tilt = 0.0
        self.cmd_pan = 0.0
        self.cmd_tilt = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_tick = self.get_clock().now()

        self.create_subscription(Vector3Stamped, 'ptu/cmd_rate', self.on_cmd, 10)
        self.pub = self.create_publisher(Vector3Stamped, 'ptu/state', 10)
        self.create_timer(1.0 / self.get_parameter('update_hz').value, self.tick)

    def on_cmd(self, msg):
        self.cmd_pan = clamp(msg.vector.x, -self.max_rate, self.max_rate)
        self.cmd_tilt = clamp(msg.vector.y, -self.max_rate, self.max_rate)
        self.last_cmd_time = self.get_clock().now()

    def slew(self, vel, cmd, dt):
        max_dv = self.max_accel * dt
        return vel + clamp(cmd - vel, -max_dv, max_dv)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now

        cmd_pan, cmd_tilt = self.cmd_pan, self.cmd_tilt
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            cmd_pan, cmd_tilt = 0.0, 0.0

        self.vel_pan = self.slew(self.vel_pan, cmd_pan, dt)
        self.vel_tilt = self.slew(self.vel_tilt, cmd_tilt, dt)

        self.pan += self.vel_pan * dt
        self.tilt += self.vel_tilt * dt

        if not self.pan_lim[0] <= self.pan <= self.pan_lim[1]:
            self.pan = clamp(self.pan, *self.pan_lim)
            self.vel_pan = 0.0
            self.get_logger().warn('pan limit switch hit', throttle_duration_sec=2.0)
        if not self.tilt_lim[0] <= self.tilt <= self.tilt_lim[1]:
            self.tilt = clamp(self.tilt, *self.tilt_lim)
            self.vel_tilt = 0.0
            self.get_logger().warn('tilt limit switch hit', throttle_duration_sec=2.0)

        out = Vector3Stamped()
        out.header.stamp = now.to_msg()
        out.vector.x = self.pan
        out.vector.y = self.tilt
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PtuSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
