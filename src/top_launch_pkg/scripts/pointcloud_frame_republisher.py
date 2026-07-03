#!/usr/bin/env python3
"""Republish PointCloud2 with a frame_id suitable for EGO-Planner."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class PointCloudFrameRepublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_frame_republisher")
        self.declare_parameter("input_topic", "/drone_0_pcl_render_node/cloud")
        self.declare_parameter("output_topic", "/drone_0_cloud_registered")
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("stamp_now", False)

        self.frame_id = self.get_parameter("frame_id").value
        self.stamp_now = _as_bool(self.get_parameter("stamp_now").value)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value, self.on_cloud, 10
        )
        self.get_logger().info(
            "republishing %s -> %s with frame_id=%s"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
                self.frame_id,
            )
        )

    def on_cloud(self, msg: PointCloud2):
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        if self.stamp_now:
            out.header.stamp = self.get_clock().now().to_msg()
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PointCloudFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
