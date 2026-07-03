#!/usr/bin/env python3
"""Turn a Unity ring PoseArray into sequential /move_base_simple/goal targets."""

import math

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class PoseArrayGoalSequencer(Node):
    def __init__(self):
        super().__init__("pose_array_goal_sequencer")
        self.declare_parameter("ring_topic", "/unity/target_rings")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("reach_xy_tolerance", 0.35)
        self.declare_parameter("reach_z_tolerance", 0.30)
        self.declare_parameter("republish_period_sec", 1.0)
        self.declare_parameter("auto_start", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.xy_tol = float(self.get_parameter("reach_xy_tolerance").value)
        self.z_tol = float(self.get_parameter("reach_z_tolerance").value)
        self.poses = []
        self.index = 0
        self.last_odom = None
        self.active = _as_bool(self.get_parameter("auto_start").value)

        self.pub_goal = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 10
        )
        self.create_subscription(PoseArray, self.get_parameter("ring_topic").value, self.on_rings, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 20)
        self.create_timer(float(self.get_parameter("republish_period_sec").value), self.on_timer)
        self.get_logger().info(
            "waiting for ring PoseArray on %s" % self.get_parameter("ring_topic").value
        )

    def on_rings(self, msg: PoseArray):
        self.poses = list(msg.poses)
        self.index = 0
        self.active = bool(self.poses)
        self.get_logger().info("loaded %d ring goals" % len(self.poses))
        self.publish_current_goal()

    def on_odom(self, msg: Odometry):
        self.last_odom = msg
        if not self.active or self.index >= len(self.poses):
            return

        target = self.poses[self.index].position
        pos = msg.pose.pose.position
        dxy = math.hypot(pos.x - target.x, pos.y - target.y)
        dz = abs(pos.z - target.z)
        if dxy <= self.xy_tol and dz <= self.z_tol:
            self.index += 1
            if self.index >= len(self.poses):
                self.active = False
                self.get_logger().info("all ring goals reached")
            else:
                self.get_logger().info("ring reached; advancing to goal %d" % self.index)
                self.publish_current_goal()

    def on_timer(self):
        if self.active:
            self.publish_current_goal()

    def publish_current_goal(self):
        if self.index >= len(self.poses):
            return
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.pose = self.poses[self.index]
        self.pub_goal.publish(goal)


def main():
    rclpy.init()
    node = PoseArrayGoalSequencer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
