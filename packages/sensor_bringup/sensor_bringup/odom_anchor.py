#!/usr/bin/env python3

# TODO do this a litle better with the static TF frames

"""
One-time odometry frame anchoring node.

This node computes a fixed transform from `map` -> `odom_frame` using:

    T_map_odom = T_map_child_from_GPS * inv(T_odom_child_from_local_odom)

The transform is computed ONCE when:
  - at least one local odometry message is received
  - at least one global odometry message is received
  - the global odometry covariance is below a configurable threshold

After initialization, the transform is frozen permanently and
broadcast continuously. It will NOT update with new GPS data.

Designed to be run multiple times for different odometry sources
(IMU odom, DVL odom, visual odom, etc.).
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation


# -----------------------------------------------------------------------------
# Utility functions
# -----------------------------------------------------------------------------

def odom_to_matrix(msg: Odometry) -> np.ndarray:
    """Convert nav_msgs/Odometry to 4x4 homogeneous transform."""

    p = msg.pose.pose.position
    q = msg.pose.pose.orientation

    T = np.eye(4)

    T[:3, :3] = Rotation.from_quat([
        q.x,
        q.y,
        q.z,
        q.w
    ]).as_matrix()

    T[0, 3] = p.x
    T[1, 3] = p.y
    T[2, 3] = p.z

    return T


def matrix_to_tf(T: np.ndarray,
                 parent: str,
                 child: str,
                 stamp):
    """Convert 4x4 transform matrix to TransformStamped."""

    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child

    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])

    q = Rotation.from_matrix(T[:3, :3]).as_quat()

    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])

    return msg


def covariance_is_valid(cov: list,
                         max_position_var: float,
                         max_yaw_var: float) -> bool:
    """
    Check GPS odometry covariance.

    Covariance layout (row-major 6x6):
      x  y  z  rx ry rz

    We gate only on:
      - x, y position variance
      - yaw (rz) variance
    """

    if len(cov) != 36:
        return False

    var_x = cov[0]
    var_y = cov[7]
    var_yaw = cov[35]

    # if var_x <= 0.0 or var_y <= 0.0 or var_yaw <= 0.0:
    #     return False
    # TODO fix later when yaw variance is available

    if var_x > max_position_var:
        return False

    if var_y > max_position_var:
        return False

    if var_yaw > max_yaw_var:
        return False

    return True


# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------

class OdomFrameAnchor(Node):

    def __init__(self):
        super().__init__("odom_frame_anchor")

        # -----------------------------
        # Parameters
        # -----------------------------

        self.declare_parameter("local_odom_topic", "/local_odom")
        self.declare_parameter("global_odom_topic", "/gps/odom")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("child_frame", "base_link")

        self.declare_parameter("publish_rate", 10.0)

        # Covariance gating (GPS only)
        self.declare_parameter("max_position_variance", 4.0)   # meters^2
        self.declare_parameter("max_yaw_variance", 0.5)        # rad^2

        # -----------------------------
        # Read parameters
        # -----------------------------

        self.local_odom_topic = self.get_parameter(
            "local_odom_topic").value
        self.global_odom_topic = self.get_parameter(
            "global_odom_topic").value

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.publish_rate = self.get_parameter("publish_rate").value

        self.max_position_var = self.get_parameter(
            "max_position_variance").value
        self.max_yaw_var = self.get_parameter(
            "max_yaw_variance").value

        # -----------------------------
        # State
        # -----------------------------

        self.local_odom = None
        self.global_odom = None

        self.T_map_odom = None

        # -----------------------------
        # ROS interfaces
        # -----------------------------

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            self.local_odom_topic,
            self.local_odom_cb,
            10)

        self.create_subscription(
            Odometry,
            self.global_odom_topic,
            self.global_odom_cb,
            10)

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_cb)

        self.get_logger().info("--------------------------------------")
        self.get_logger().info(" Odom Frame Anchor Node")
        self.get_logger().info(f" map frame : {self.map_frame}")
        self.get_logger().info(f" odom frame: {self.odom_frame}")
        self.get_logger().info(f" child     : {self.child_frame}")
        self.get_logger().info("--------------------------------------")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def local_odom_cb(self, msg: Odometry):
        if self.local_odom is None:
            self.local_odom = msg

    def global_odom_cb(self, msg: Odometry):
        if self.global_odom is not None:
            return

        if not covariance_is_valid(
            msg.pose.covariance,
            self.max_position_var,
            self.max_yaw_var,
        ):
            self.get_logger().warn(
                "GPS covariance not acceptable yet — waiting...",
                throttle_duration_sec=5.0,
            )
            return

        self.global_odom = msg

        self.get_logger().info(
            "Accepted GPS odometry — covariance within limits"
        )

    # ------------------------------------------------------------------

    def initialize_transform(self):

        T_map_child = odom_to_matrix(self.global_odom)
        T_odom_child = odom_to_matrix(self.local_odom)

        self.T_map_odom = T_map_child @ np.linalg.inv(T_odom_child)

        self.get_logger().info(
            f"Initialized static transform: {self.map_frame} -> {self.odom_frame}"
        )

    # ------------------------------------------------------------------

    def timer_cb(self):

        if self.T_map_odom is None:
            if self.local_odom is None or self.global_odom is None:
                return

            self.initialize_transform()

        tf_msg = matrix_to_tf(
            self.T_map_odom,
            self.map_frame,
            self.odom_frame,
            self.get_clock().now().to_msg()
        )

        self.tf_broadcaster.sendTransform(tf_msg)


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    rclpy.init()
    node = OdomFrameAnchor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
