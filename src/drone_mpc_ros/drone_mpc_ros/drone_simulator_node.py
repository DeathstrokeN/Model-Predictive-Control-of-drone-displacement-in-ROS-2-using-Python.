"""Simple drone simulator node.

This node simulates only translational motion in 3D.
It subscribes to the acceleration command produced by the MPC and publishes odometry.

The purpose is educational: keep the physics simple so the control logic remains clear.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped
from nav_msgs.msg import Odometry


class DroneSimulatorNode(Node):
    """A minimal 3D point-mass drone simulator."""

    def __init__(self) -> None:
        super().__init__('drone_simulator_node')

        self.dt = 0.02  # 50 Hz simulation loop
        self.drag = 0.15  # light linear damping for realism

        # State: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        self.current_accel_cmd = np.zeros(3)

        self.odom_pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.cmd_sub = self.create_subscription(
            AccelStamped,
            '/drone/cmd_accel',
            self.cmd_callback,
            10,
        )
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info('Drone simulator started.')

    def cmd_callback(self, msg: AccelStamped) -> None:
        self.current_accel_cmd = np.array([
            msg.accel.linear.x,
            msg.accel.linear.y,
            msg.accel.linear.z,
        ], dtype=float)

    def step(self) -> None:
        pos = self.state[0:3]
        vel = self.state[3:6]

        # Simple translational model with damping.
        accel = self.current_accel_cmd - self.drag * vel

        vel = vel + self.dt * accel
        pos = pos + self.dt * vel

        self.state[0:3] = pos
        self.state[3:6] = vel

        self.publish_odometry()

    def publish_odometry(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(self.state[0])
        msg.pose.pose.position.y = float(self.state[1])
        msg.pose.pose.position.z = float(self.state[2])

        msg.twist.twist.linear.x = float(self.state[3])
        msg.twist.twist.linear.y = float(self.state[4])
        msg.twist.twist.linear.z = float(self.state[5])

        self.odom_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
