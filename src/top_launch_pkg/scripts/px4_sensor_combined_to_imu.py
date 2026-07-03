#!/usr/bin/env python3
"""Convert PX4 SensorCombined FRD IMU samples into ROS sensor_msgs/Imu FLU."""

import rclpy
from px4_msgs.msg import SensorCombined
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class Px4SensorCombinedToImu(Node):
    def __init__(self):
        super().__init__("px4_sensor_combined_to_imu")
        self.declare_parameter("input_topic", "/fmu/out/sensor_combined")
        self.declare_parameter("output_topic", "/sim/livox/imu")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("stamp_from_px4", False)
        self.declare_parameter("acceleration_scale", 1.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.stamp_from_px4 = _as_bool(self.get_parameter("stamp_from_px4").value)
        self.acc_scale = float(self.get_parameter("acceleration_scale").value)
        self.pub = self.create_publisher(Imu, self.get_parameter("output_topic").value, 50)
        self.create_subscription(
            SensorCombined, self.get_parameter("input_topic").value, self.on_sensor, 50
        )
        self.get_logger().info(
            "PX4 SensorCombined -> Imu: %s -> %s, acc_scale=%g"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
                self.acc_scale,
            )
        )

    def on_sensor(self, msg: SensorCombined):
        imu = Imu()
        if self.stamp_from_px4:
            seconds = int(msg.timestamp // 1000000)
            nanoseconds = int((msg.timestamp % 1000000) * 1000)
            imu.header.stamp.sec = seconds
            imu.header.stamp.nanosec = nanoseconds
        else:
            imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        # PX4 reports body FRD; ROS consumers in this stack expect body FLU.
        imu.angular_velocity.x = float(msg.gyro_rad[0])
        imu.angular_velocity.y = -float(msg.gyro_rad[1])
        imu.angular_velocity.z = -float(msg.gyro_rad[2])
        imu.linear_acceleration.x = float(msg.accelerometer_m_s2[0]) * self.acc_scale
        imu.linear_acceleration.y = -float(msg.accelerometer_m_s2[1]) * self.acc_scale
        imu.linear_acceleration.z = -float(msg.accelerometer_m_s2[2]) * self.acc_scale
        imu.orientation_covariance[0] = -1.0
        self.pub.publish(imu)


def main():
    rclpy.init()
    node = Px4SensorCombinedToImu()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
