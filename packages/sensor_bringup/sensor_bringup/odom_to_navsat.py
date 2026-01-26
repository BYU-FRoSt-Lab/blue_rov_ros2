#!/usr/bin/env python3

"""
Odometry → NavSatFix test conversion node.

Purpose:
--------
This node converts an incoming nav_msgs/Odometry message into a
sensor_msgs/NavSatFix message using:

  • A known geographic origin (lat / lon / alt)
  • TF transforms between the odometry frame and the map frame

This is intended ONLY as a validation and testing tool to confirm that
odom → map → ENU → LLA conversions are behaving as expected.

It does NOT perform Earth modeling beyond a flat-Earth ENU assumption.

Coordinate assumptions:
------------------------
• map frame is ENU
    +X east
    +Y north
    +Z up

• origin latitude / longitude defines map (0,0,0)

• Odometry pose is expressed in its own frame

Transform chain:
----------------

    odom_msg.pose.pose  (child frame)
            |
            v
      odom_frame
            |
            v
         map frame   <-- origin lat/lon

The node computes the position of the odometry sensor expressed in the
map frame and converts ENU meters → latitude/longitude.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from scipy.spatial.transform import Rotation


# -----------------------------------------------------------------------------
# Earth constants (WGS84)
# -----------------------------------------------------------------------------

EARTH_RADIUS_EQUATOR = 6378137.0  # meters


# -----------------------------------------------------------------------------
# Utilities
# -----------------------------------------------------------------------------

def odom_pose_to_matrix(msg: Odometry) -> np.ndarray:
    """Convert odometry pose to 4x4 transform."""

    p = msg.pose.pose.position
    q = msg.pose.pose.orientation

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([
        q.x, q.y, q.z, q.w
    ]).as_matrix()

    T[0, 3] = p.x
    T[1, 3] = p.y
    T[2, 3] = p.z

    return T


# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------

class OdomToNavSatFix(Node):

    def __init__(self):
        super().__init__("odom_to_navsatfix")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------

        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("output_topic", "fix")

        self.declare_parameter("origin.latitude", 0.0)
        self.declare_parameter("origin.longitude", 0.0)
        self.declare_parameter("origin.altitude", 0.0)

        self.declare_parameter("map_frame", "map")

        # ------------------------------------------------------------------

        self.odom_topic = self.get_parameter("odom_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.origin_lat = math.radians(
            self.get_parameter("origin.latitude").value
        )
        self.origin_lon = math.radians(
            self.get_parameter("origin.longitude").value
        )
        self.origin_alt = self.get_parameter("origin.altitude").value

        self.map_frame = self.get_parameter("map_frame").value

        # ------------------------------------------------------------------
        # TF
        # ------------------------------------------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            10,
        )

        self.pub = self.create_publisher(
            NavSatFix,
            self.output_topic,
            10,
        )

        self.get_logger().info("--------------------------------------")
        self.get_logger().info(" Odometry → NavSatFix test node")
        self.get_logger().info(f" Origin lat: {math.degrees(self.origin_lat)}")
        self.get_logger().info(f" Origin lon: {math.degrees(self.origin_lon)}")
        self.get_logger().info(f" Map frame : {self.map_frame}")
        self.get_logger().info("--------------------------------------")

    # ------------------------------------------------------------------

    def odom_cb(self, msg: Odometry):

        odom_frame = msg.header.frame_id
        child_frame = msg.child_frame_id

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                odom_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        # --------------------------------------------------------------
        # Build transforms
        # --------------------------------------------------------------

        # map -> odom
        T_map_odom = np.eye(4)
        T_map_odom[:3, :3] = Rotation.from_quat([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]).as_matrix()

        T_map_odom[0, 3] = tf.transform.translation.x
        T_map_odom[1, 3] = tf.transform.translation.y
        T_map_odom[2, 3] = tf.transform.translation.z

        # odom -> child (sensor)
        T_odom_child = odom_pose_to_matrix(msg)

        # map -> child
        T_map_child = T_map_odom @ T_odom_child

        # --------------------------------------------------------------
        # ENU position
        # --------------------------------------------------------------

        east = T_map_child[0, 3]
        north = T_map_child[1, 3]
        up = T_map_child[2, 3]

        # --------------------------------------------------------------
        # Flat-earth ENU → LLA
        # --------------------------------------------------------------

        d_lat = north / EARTH_RADIUS_EQUATOR
        d_lon = east / (EARTH_RADIUS_EQUATOR * math.cos(self.origin_lat))

        lat = self.origin_lat + d_lat
        lon = self.origin_lon + d_lon
        alt = self.origin_alt + up

        # --------------------------------------------------------------
        # Publish NavSatFix
        # --------------------------------------------------------------

        fix = NavSatFix()
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = self.map_frame

        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = math.degrees(lat)
        fix.longitude = math.degrees(lon)
        fix.altitude = alt

        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub.publish(fix)


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    rclpy.init()
    node = OdomToNavSatFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
