#!/usr/bin/env python3
# Multi-waypoint sequencer for EGO + px4_offboard stack.
#
# 订阅 /Odometry, 按顺序把 waypoints 发布到 /move_base_simple/goal。
# 每个航点 (XY/Z 容差 + 速度小 + 持续 settle 秒) 到达后立即发下一个,
# 中间点不悬停; 最后一个点发出后 sequencer 不再插手, 让 px4_offboard 自己
# 走 FINAL_CORRECTION -> MID360 精对 -> LAND -> DISARM。
#
# 阈值与 px4_offboard 的 ego_goal_reach_* 一致, 保证 sequencer 切点的时机
# 不晚于 px4_offboard 进入 FINAL_CORRECTION + 5s HOLD 倒计时, 避免误降落。
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def parse_waypoints(s):
    pts = []
    for chunk in s.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.split(',')
        if len(parts) != 3:
            raise ValueError("bad waypoint '%s', expected 'x,y,z'" % chunk)
        pts.append(tuple(float(v) for v in parts))
    if not pts:
        raise ValueError('need at least one waypoint')
    return pts


class MultiWaypointPublisher(Node):
    def __init__(self, waypoints, xy_tol, z_tol, vel_tol, settle_sec,
                 odom_topic, goal_topic, frame_id, wait_subscribers):
        super().__init__('multi_waypoint_publisher')
        self.waypoints = waypoints
        self.xy_tol = xy_tol
        self.z_tol = z_tol
        self.vel_tol = vel_tol
        self.settle_sec = settle_sec
        self.frame_id = frame_id
        self.wait_subscribers = wait_subscribers
        self.idx = 0
        self.close_since = None
        self.first_published = False

        # goal: 沿用 RViz 默认的 reliable 队列, ego_planner / px4_offboard 都订
        self.pub = self.create_publisher(PoseStamped, goal_topic, 10)
        odom_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, odom_qos)
        self.startup_timer = self.create_timer(0.5, self.try_publish_first)
        self.get_logger().info(
            'Multi-waypoint sequencer: %d pts, odom=%s, goal=%s, '
            'xy_tol=%.2f z_tol=%.2f vel_tol=%.2f settle=%.1fs' % (
                len(waypoints), odom_topic, goal_topic,
                xy_tol, z_tol, vel_tol, settle_sec))
        for i, p in enumerate(waypoints):
            self.get_logger().info(
                '  [%d] (%.2f, %.2f, %.2f)' % (i + 1, p[0], p[1], p[2]))

    def try_publish_first(self):
        if self.first_published:
            return
        n = self.pub.get_subscription_count()
        if n < self.wait_subscribers:
            self.get_logger().info(
                'Waiting for %d subscribers on goal topic (have %d)' % (
                    self.wait_subscribers, n),
                throttle_duration_sec=2.0)
            return
        self.publish_current()
        self.first_published = True
        self.startup_timer.cancel()

    def publish_current(self):
        x, y, z = self.waypoints[self.idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.close_since = None
        is_last = (self.idx == len(self.waypoints) - 1)
        self.get_logger().info(
            '[WP %d/%d] publish -> (%.2f, %.2f, %.2f)%s' % (
                self.idx + 1, len(self.waypoints), x, y, z,
                '  (final: px4_offboard takes over)' if is_last else ''))

    def on_odom(self, msg):
        # 还没开始, 或最后一个已经发出 -> 交给 px4_offboard
        if not self.first_published:
            return
        if self.idx >= len(self.waypoints) - 1:
            return
        x, y, z = self.waypoints[self.idx]
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        d_xy = math.hypot(px - x, py - y)
        d_z = abs(pz - z)
        v = math.sqrt(vx * vx + vy * vy + vz * vz)
        now = time.monotonic()
        if d_xy <= self.xy_tol and d_z <= self.z_tol and v <= self.vel_tol:
            if self.close_since is None:
                self.close_since = now
                self.get_logger().info(
                    '[WP %d/%d] within tol (d_xy=%.2f d_z=%.2f v=%.2f), '
                    'settling...' % (
                        self.idx + 1, len(self.waypoints), d_xy, d_z, v))
            elif now - self.close_since >= self.settle_sec:
                self.idx += 1
                self.publish_current()
        else:
            self.close_since = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints', required=True,
                        help='";"-separated "x,y,z" triplets in ENU map frame')
    parser.add_argument('--xy_tol', type=float, default=0.35)
    parser.add_argument('--z_tol', type=float, default=0.25)
    parser.add_argument('--vel_tol', type=float, default=0.20)
    parser.add_argument('--settle_sec', type=float, default=1.0)
    parser.add_argument('--odom_topic', default='/Odometry')
    parser.add_argument('--goal_topic', default='/move_base_simple/goal')
    parser.add_argument('--frame_id', default='map')
    parser.add_argument('--wait_subscribers', type=int, default=2)
    # ros2 launch 会附 --ros-args 之类的参数, 用 parse_known_args 忽略
    args, _ = parser.parse_known_args()

    waypoints = parse_waypoints(args.waypoints)
    rclpy.init()
    node = MultiWaypointPublisher(
        waypoints=waypoints,
        xy_tol=args.xy_tol, z_tol=args.z_tol,
        vel_tol=args.vel_tol, settle_sec=args.settle_sec,
        odom_topic=args.odom_topic, goal_topic=args.goal_topic,
        frame_id=args.frame_id, wait_subscribers=args.wait_subscribers)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
