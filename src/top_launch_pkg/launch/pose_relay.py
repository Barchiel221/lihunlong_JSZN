#!/usr/bin/env python3
# pose_relay (改动 H, 仿真里程碑1)。据《路线B 指南》第 6.3。
#
# 把 Gazebo/Unity 的真值位姿转发两路:
#   /quad_0/lidar_slam/odom  给 Unity
#   /Odometry  (frame=camera_init) 给 EGO 当定位 —— 里程碑1 用真值绕开 FAST-LIO
# 里程碑2 关掉 /Odometry 这一路 (--no_lio), 交给 FAST-LIO。
#
# ★★★ 坐标符号必须在 rviz 现场标定 ★★★
# 真值原始系随插件/机型而变, map 是 NWU(指南红线3)。先给恒等, 让无人机沿 Gazebo +x
# 平移确认 EGO map 里朝对应方向动、Unity 画面一致, 再固化符号。别照抄符号。
import argparse
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

try:
    from gazebo_msgs.msg import ModelStates
except ImportError:
    ModelStates = None


class PoseRelay(Node):
    def __init__(self, drone_model, publish_lio, origin):
        super().__init__('pose_relay')
        self.drone_model = drone_model
        self.origin = origin
        self.pub_unity = self.create_publisher(Odometry, '/quad_0/lidar_slam/odom', 10)
        self.pub_lio = self.create_publisher(Odometry, '/Odometry', 10) if publish_lio else None
        if ModelStates is None:
            self.get_logger().error('gazebo_msgs not available; cannot subscribe /gazebo/model_states')
            return
        self.create_subscription(ModelStates, '/gazebo/model_states', self.cb, 10)
        self.get_logger().info(
            'pose_relay: model=%s publish_lio=%s (★坐标符号须 rviz 现场标定★)'
            % (drone_model, publish_lio))

    def cb(self, msg):
        if self.drone_model not in msg.name:
            return
        i = msg.name.index(self.drone_model)
        p = msg.pose[i].position
        # ★ 恒等转换占位: 标定后在此填 ENU/FLU -> NWU 的符号变换。
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'camera_init'
        od.child_frame_id = 'body'
        od.pose.pose = msg.pose[i]
        od.pose.pose.position.x = p.x - self.origin[0]
        od.pose.pose.position.y = p.y - self.origin[1]
        od.pose.pose.position.z = p.z - self.origin[2]
        od.twist.twist = msg.twist[i]
        self.pub_unity.publish(od)
        if self.pub_lio:
            self.pub_lio.publish(od)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone_model', default='iris', help='SITL 机型名(gazebo model)')
    parser.add_argument('--no_lio', action='store_true',
                        help='里程碑2: 不发 /Odometry(交给 FAST-LIO)')
    parser.add_argument('--origin', default='0,0,0', help='起飞点相对 Unity 原点偏置 x,y,z')
    args, _ = parser.parse_known_args()
    origin = tuple(float(v) for v in args.origin.split(','))

    rclpy.init()
    node = PoseRelay(args.drone_model, not args.no_lio, origin)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
