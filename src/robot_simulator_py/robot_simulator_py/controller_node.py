import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(rad):
    # wrap angle to [-pi, pi]
    return math.atan2(math.sin(rad), math.cos(rad))


class Mode(Enum):
    DRIVE = 0
    TURN  = 1


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('linear_speed', 0.3)     # m/s
        self.declare_parameter('angular_speed', 0.6)    # rad/s
        self.declare_parameter('dist_tol', 0.02)        # m
        self.declare_parameter('yaw_tol',  math.radians(2.0))  # rad

        self.side_length   = float(self.get_parameter('side_length').value)
        self.linear_speed  = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.dist_tol      = float(self.get_parameter('dist_tol').value)
        self.yaw_tol       = float(self.get_parameter('yaw_tol').value)

        # --- pubs / tf ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # --- state ---
        self.mode = Mode.DRIVE
        self.edge_count = 0              # 0..3, then stop
        self.start_x = None              # starting point of current segment
        self.start_y = None
        self.target_yaw = None           # for TURN state

        self.last_tf_warn = 0.0          # throttle TF warnings
        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz

        self.get_logger().info('controller_node up: driving a 2x2 square using TF odom->base_link')

    # ---- helpers ----
    def get_pose(self):
        try:
            # latest available transform
            t = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = yaw_from_quat(t.transform.rotation)
            return x, y, yaw, True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # throttle noise
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - self.last_tf_warn > 1.0:
                self.get_logger().warn(f'Waiting for TF odom->base_link: {e}')
                self.last_tf_warn = now
            return 0.0, 0.0, 0.0, False

    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    # ---- main loop ----
    def on_timer(self):
        x, y, yaw, ok = self.get_pose()
        if not ok:
            self.stop()
            return

        if self.mode == Mode.DRIVE:
            if self.start_x is None:
                # starting a new edge: remember where we began
                self.start_x, self.start_y = x, y

            # distance traveled along current edge
            dx = x - self.start_x
            dy = y - self.start_y
            dist = math.hypot(dx, dy)

            if dist >= self.side_length - self.dist_tol:
                # reached edge → switch to TURN
                self.stop()
                self.mode = Mode.TURN
                self.target_yaw = angle_wrap(yaw + math.pi / 2.0)  # +90 deg
                return

            # drive straight (in base frame: forward x)
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

        elif self.mode == Mode.TURN:
            # rotate in place toward target_yaw
            err = angle_wrap(self.target_yaw - yaw)

            if abs(err) <= self.yaw_tol:
                # done turning → next edge
                self.stop()
                self.mode = Mode.DRIVE
                self.edge_count += 1
                self.start_x, self.start_y = None, None

                if self.edge_count >= 4:
                    self.get_logger().info('Square complete. Stopping.')
                    # optionally: cancel timer to stop node activity
                    self.timer.cancel()
                return

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed * (1.0 if err > 0.0 else -1.0)
            self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
