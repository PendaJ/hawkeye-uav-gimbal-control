"""Adapt Hawkeye /cam_vel and Gazebo gimbal joint feedback.

Conventions retained from the existing Hawkeye SS-PD controller:
  /cam_vel (geometry_msgs/Twist): angular.x=pitch, angular.y=yaw [rad/s]
  /current_orientation (geometry_msgs/Vector3): x=pitch, z=yaw [degrees]

No optimizer or perception node runs here; this is a command/feedback adapter.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class HawkeyeGimbalAdapter(Node):
    def __init__(self):
        super().__init__('hawkeye_gimbal_adapter')
        self.declare_parameter('max_velocity_rad_s', 0.2)
        self.declare_parameter('command_timeout_s', 0.5)
        self.max_velocity = abs(float(self.get_parameter('max_velocity_rad_s').value))
        self.timeout = max(0.05, float(self.get_parameter('command_timeout_s').value))

        self.yaw_pub = self.create_publisher(
            Float64, '/model/x3/joint/gimbal_yaw_joint/cmd_vel', 10)
        self.pitch_pub = self.create_publisher(
            Float64, '/model/x3/joint/gimbal_pitch_joint/cmd_vel', 10)
        self.orientation_pub = self.create_publisher(
            Vector3, '/current_orientation', 10)

        self.create_subscription(Twist, '/cam_vel', self._on_command, 10)
        self.create_subscription(
            JointState, '/model/x3/gimbal_joint_state', self._on_joint_state, 10)

        self._yaw_command = 0.0
        self._pitch_command = 0.0
        self._last_command_monotonic = -1.0
        self.create_timer(0.02, self._publish_command)  # 50 Hz bridge output
        self.get_logger().info(
            f'Gimbal adapter running: |joint rate| <= {self.max_velocity:.3f} rad/s; '
            f'deadman={self.timeout:.2f}s')

    def _clamp(self, x):
        return max(-self.max_velocity, min(self.max_velocity, x))

    def _on_command(self, msg):
        pitch = float(msg.angular.x)
        yaw = float(msg.angular.y)
        if not (math.isfinite(pitch) and math.isfinite(yaw)):
            self.get_logger().warning('Ignoring non-finite /cam_vel command')
            return
        self._pitch_command = self._clamp(pitch)
        self._yaw_command = self._clamp(yaw)
        self._last_command_monotonic = time.monotonic()

    def _publish_command(self):
        fresh = (self._last_command_monotonic >= 0 and
                 (time.monotonic() - self._last_command_monotonic) < self.timeout)
        self.yaw_pub.publish(Float64(data=self._yaw_command if fresh else 0.0))
        self.pitch_pub.publish(Float64(data=self._pitch_command if fresh else 0.0))

    def _on_joint_state(self, msg):
        angles = dict(zip(msg.name, msg.position))
        if 'gimbal_yaw_joint' not in angles or 'gimbal_pitch_joint' not in angles:
            return
        output = Vector3()
        output.x = math.degrees(float(angles['gimbal_pitch_joint']))
        output.y = 0.0
        output.z = math.degrees(float(angles['gimbal_yaw_joint']))
        self.orientation_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = HawkeyeGimbalAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
