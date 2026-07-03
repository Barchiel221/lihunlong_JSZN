#!/usr/bin/env python3
"""Relay Gazebo ground truth into the frames used by Unity and EGO."""

import math
from typing import Iterable, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # x, y, z, w


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _mat_vec_mul(m, v: Vector3) -> Vector3:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _mat_mul(a, b):
    return [
        [
            a[row][0] * b[0][col]
            + a[row][1] * b[1][col]
            + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def _mat_transpose(m):
    return [[m[col][row] for col in range(3)] for row in range(3)]


def _quat_to_mat(q: Quaternion):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _mat_to_quat(m) -> Quaternion:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def _basis_matrix(mode: str):
    if mode == "passthrough":
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    if mode == "enu_to_nwu":
        return [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    if mode == "ned_to_nwu":
        return [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    raise ValueError(f"unsupported transform_mode '{mode}'")


class PoseRelay(Node):
    def __init__(self):
        super().__init__("pose_relay")
        self.declare_parameter("model_name", "iris")
        self.declare_parameter("input_type", "gazebo_model_states")
        self.declare_parameter("input_topic", "/gazebo/model_states")
        self.declare_parameter("unity_odom_topic", "/quad_0/lidar_slam/odom")
        self.declare_parameter("lio_odom_topic", "/Odometry")
        self.declare_parameter("publish_lio_odom", True)
        self.declare_parameter("transform_mode", "passthrough")
        self.declare_parameter("origin", [0.0, 0.0, 0.0])
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("child_frame_id", "body")

        self.model_name = self.get_parameter("model_name").value
        self.input_type = self.get_parameter("input_type").value
        self.publish_lio = _as_bool(self.get_parameter("publish_lio_odom").value)
        self.transform_mode = self.get_parameter("transform_mode").value
        self.origin = self._read_origin(self.get_parameter("origin").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.basis = _basis_matrix(self.transform_mode)
        self.basis_t = _mat_transpose(self.basis)
        self.warned_missing_model = False

        self.pub_unity = self.create_publisher(
            Odometry, self.get_parameter("unity_odom_topic").value, 10
        )
        self.pub_lio = None
        if self.publish_lio:
            self.pub_lio = self.create_publisher(
                Odometry, self.get_parameter("lio_odom_topic").value, 10
            )
        input_topic = self.get_parameter("input_topic").value
        self._create_input_subscription(input_topic)

        self.get_logger().info(
            "pose_relay input=%s:%s model=%s mode=%s publish_lio=%s origin=%s"
            % (
                self.input_type,
                input_topic,
                self.model_name,
                self.transform_mode,
                self.publish_lio,
                self.origin,
            )
        )

    @staticmethod
    def _read_origin(value: Iterable[float]) -> Vector3:
        vals = list(value)
        if len(vals) != 3:
            raise ValueError("origin must contain exactly three numbers")
        return (float(vals[0]), float(vals[1]), float(vals[2]))

    def _transform_vector(self, v: Vector3) -> Vector3:
        out = _mat_vec_mul(self.basis, v)
        return (out[0] - self.origin[0], out[1] - self.origin[1], out[2] - self.origin[2])

    def _rotate_vector(self, v: Vector3) -> Vector3:
        return _mat_vec_mul(self.basis, v)

    def _transform_quaternion(self, q: Quaternion) -> Quaternion:
        r_in = _quat_to_mat(q)
        r_out = _mat_mul(_mat_mul(self.basis, r_in), self.basis_t)
        return _mat_to_quat(r_out)

    def _create_input_subscription(self, input_topic: str):
        if self.input_type == "gazebo_model_states":
            try:
                msg_type = get_message("gazebo_msgs/msg/ModelStates")
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                raise RuntimeError(
                    "input_type=gazebo_model_states requires gazebo_msgs. "
                    "Install the Gazebo ROS message package or use input_type=nav_msgs_odometry."
                ) from exc
            self.create_subscription(msg_type, input_topic, self.on_model_states, 10)
        elif self.input_type == "nav_msgs_odometry":
            self.create_subscription(Odometry, input_topic, self.on_odometry, 10)
        elif self.input_type == "px4_vehicle_odometry":
            msg_type = get_message("px4_msgs/msg/VehicleOdometry")
            self.create_subscription(msg_type, input_topic, self.on_px4_vehicle_odometry, 10)
        else:
            raise ValueError(
                "input_type must be gazebo_model_states, nav_msgs_odometry, or px4_vehicle_odometry"
            )

    def publish_transformed(
        self,
        position: Vector3,
        orientation: Quaternion,
        linear_velocity: Vector3,
        angular_velocity: Vector3,
    ):
        pos = self._transform_vector(position)
        quat = self._transform_quaternion(orientation)
        lin = self._rotate_vector(linear_velocity)
        ang = self._rotate_vector(angular_velocity)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = lin[0]
        odom.twist.twist.linear.y = lin[1]
        odom.twist.twist.linear.z = lin[2]
        odom.twist.twist.angular.x = ang[0]
        odom.twist.twist.angular.y = ang[1]
        odom.twist.twist.angular.z = ang[2]

        self.pub_unity.publish(odom)
        if self.pub_lio is not None:
            self.pub_lio.publish(odom)

    def on_model_states(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            if not self.warned_missing_model:
                self.get_logger().warn(
                    "model '%s' not found in /gazebo/model_states; available=%s"
                    % (self.model_name, ", ".join(msg.name[:8]))
                )
                self.warned_missing_model = True
            return

        pose = msg.pose[index]
        twist = msg.twist[index]
        self.publish_transformed(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            (twist.linear.x, twist.linear.y, twist.linear.z),
            (twist.angular.x, twist.angular.y, twist.angular.z),
        )

    def on_odometry(self, msg: Odometry):
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.publish_transformed(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            (twist.linear.x, twist.linear.y, twist.linear.z),
            (twist.angular.x, twist.angular.y, twist.angular.z),
        )

    def on_px4_vehicle_odometry(self, msg):
        # px4_msgs/VehicleOdometry quaternion order is w, x, y, z.
        self.publish_transformed(
            (float(msg.position[0]), float(msg.position[1]), float(msg.position[2])),
            (float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0])),
            (float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])),
            (
                float(msg.angular_velocity[0]),
                float(msg.angular_velocity[1]),
                float(msg.angular_velocity[2]),
            ),
        )


def main():
    rclpy.init()
    node = PoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
