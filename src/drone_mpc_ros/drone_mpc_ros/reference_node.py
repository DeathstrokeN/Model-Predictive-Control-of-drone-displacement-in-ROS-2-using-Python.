"""Reference trajectory publisher for the drone MPC demo."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ReferenceNode(Node):
    """Publish a smooth reference position for the drone.

    The reference is deliberately simple so the MPC behavior is easy to visualize:
    - initial move toward (1, 1, 1)
    - then a slow circle in the horizontal plane
    """

    def __init__(self) -> None:
        super().__init__('reference_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/drone/reference', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0

        self.get_logger().info('Reference node started.')

    def timer_callback(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        if self.t < 5.0:
            # Smooth approach toward a setpoint.
            msg.pose.position.x = 0.2 * self.t
            msg.pose.position.y = 0.2 * self.t
            msg.pose.position.z = 0.2 * self.t
        else:
            # Then keep altitude and draw a small circle.
            tau = self.t - 5.0
            radius = 0.5
            omega = 0.25
            msg.pose.position.x = 1.0 + radius * math.cos(omega * tau)
            msg.pose.position.y = 1.0 + radius * math.sin(omega * tau)
            msg.pose.position.z = 1.0

        self.publisher_.publish(msg)
        self.t += 0.1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
