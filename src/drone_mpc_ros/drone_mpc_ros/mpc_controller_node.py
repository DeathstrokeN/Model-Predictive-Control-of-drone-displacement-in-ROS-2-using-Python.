"""ROS 2 node that runs the MPC controller for drone displacement control."""

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, AccelStamped
from nav_msgs.msg import Odometry

from drone_mpc_ros.mpc_core import DroneMPC
from drone_mpc_ros.utils import CsvLogger


class MPCControllerNode(Node):
    """Compute optimal acceleration commands from the current state and reference."""

    def __init__(self) -> None:
        super().__init__('mpc_controller_node')

        # Controller settings.
        self.dt = 0.1
        self.controller = DroneMPC(
            dt=self.dt,
            horizon=20,
            q_position=20.0,
            q_velocity=2.0,
            r_acceleration=0.1,
            p_terminal=25.0,
            max_acceleration=2.5,
            max_velocity=3.0,
        )

        # Last known system state and reference.
        self.state = np.zeros(6)
        self.reference = np.zeros(6)
        self.have_odom = False
        self.have_reference = False

        self.reference_sub = self.create_subscription(
            PoseStamped,
            '/drone/reference',
            self.reference_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(AccelStamped, '/drone/cmd_accel', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.start_time = self.get_clock().now()

        self.logger_csv = CsvLogger(
            'drone_mpc_log.csv',
            [
                'time',
                'xref', 'yref', 'zref',
                'x', 'y', 'z',
                'vx', 'vy', 'vz',
                'ax_cmd', 'ay_cmd', 'az_cmd',
            ],
        )

        self.get_logger().info('MPC controller node started.')

    def reference_callback(self, msg: PoseStamped) -> None:
        self.reference[0] = msg.pose.position.x
        self.reference[1] = msg.pose.position.y
        self.reference[2] = msg.pose.position.z
        self.reference[3:6] = 0.0  # desired terminal velocity is zero in this simple demo
        self.have_reference = True

    def odom_callback(self, msg: Odometry) -> None:
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = msg.pose.pose.position.z
        self.state[3] = msg.twist.twist.linear.x
        self.state[4] = msg.twist.twist.linear.y
        self.state[5] = msg.twist.twist.linear.z
        self.have_odom = True

    def control_loop(self) -> None:
        if not (self.have_odom and self.have_reference):
            return

        # Solve the QP and get the first optimal acceleration.
        u = self.controller.solve(self.state.copy(), self.reference.copy())

        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.accel.linear.x = float(u[0])
        msg.accel.linear.y = float(u[1])
        msg.accel.linear.z = float(u[2])
        self.cmd_pub.publish(msg)

        # Log the run to a CSV for later plotting and analysis.
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.logger_csv.write_row([
            elapsed,
            self.reference[0], self.reference[1], self.reference[2],
            self.state[0], self.state[1], self.state[2],
            self.state[3], self.state[4], self.state[5],
            u[0], u[1], u[2],
        ])

        self.get_logger().info(
            f"ref=({self.reference[0]:.2f}, {self.reference[1]:.2f}, {self.reference[2]:.2f}) | "
            f"pos=({self.state[0]:.2f}, {self.state[1]:.2f}, {self.state[2]:.2f}) | "
            f"cmd=({u[0]:.2f}, {u[1]:.2f}, {u[2]:.2f})"
        )

    def destroy_node(self):
        self.logger_csv.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
