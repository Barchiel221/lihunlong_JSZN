#!/usr/bin/env python3
# Mission executor for EGO + px4_offboard stack (改动 C, 国赛代码改动方案).
#
# 读 mission.yaml, 按顺序把 waypoints 发布到 /move_base_simple/goal, 替代 RViz 手点 goal
# (规则要求全程自主, 不能依赖地面站)。相对 multi_waypoint_publisher.py 增加三件事:
#
#   1. fly_through 到点判定: waypoint 带 fly_through:true 时, 判定退化为 d_xy<=xy_tol
#      (不查 z/速度/settle), 穿环与引导航点用它, 直接穿过不悬停。
#   2. 航段参数下发: waypoint 带 segment_params 时, 发 goal 之前先通过
#      /<ego_node>/set_parameters 服务把 max_vel / virtual_ceil / virtual_ground 推给
#      planner (依赖改动 A/B 的运行时回调), 等应答成功再发 goal。
#   3. 失败兜底: 参数服务重试 3 次仍失败 -> ERROR 日志 + 继续任务(沿用旧参数),
#      不阻塞、不失控(低位环拿不到点时最终由 px4_offboard 状态超时 LAND 兜底)。
#
# 最后一个航点语义不变: 发出后本节点不再插手, 交 px4_offboard 走
# FINAL_CORRECTION -> MID360 精对 -> LAND -> DISARM (已实飞验证链路)。
#
# 坐标: ENU, frame=map, 与 RViz 2D Goal Pose 一致。单线程 executor: control_tick 定时器
# 与 on_odom 订阅回调同线程, 参数服务用 call_async + 跨 tick 轮询 future, 不会死锁。
import argparse
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# segment_params 里的短名 -> ego planner 实际参数名。max_vel 需同时设两处(manager 分配
# 轨迹时长 + optimization feasibility 代价), 见改动 B。
_PARAM_MAP = {
    'max_vel': ['manager/max_vel', 'optimization/max_vel'],
    'virtual_ceil_height': ['grid_map/virtual_ceil_height'],
    'virtual_ground_height': ['grid_map/virtual_ground_height'],
    'virtual_ground_enable_height': ['grid_map/virtual_ground_enable_height'],
}


def load_mission(path):
    with open(path, 'r') as f:
        doc = yaml.safe_load(f)
    if not doc or 'waypoints' not in doc or not doc['waypoints']:
        raise ValueError("mission file '%s' has no waypoints" % path)
    dft = doc.get('default_tolerance', {}) or {}
    dft_xy = float(dft.get('xy', 0.35))
    dft_z = float(dft.get('z', 0.25))
    wps = []
    for i, w in enumerate(doc['waypoints']):
        if 'pos' not in w or len(w['pos']) != 3:
            raise ValueError('waypoint %d missing pos:[x,y,z]' % (i + 1))
        tol = w.get('tolerance', {}) or {}
        wps.append({
            'pos': tuple(float(v) for v in w['pos']),
            'fly_through': bool(w.get('fly_through', False)),
            'tol_xy': float(tol.get('xy', dft_xy)),
            'tol_z': float(tol.get('z', dft_z)),
            'segment_params': w.get('segment_params') or None,
        })
    return wps


class MissionExecutor(Node):
    def __init__(self, waypoints, args):
        super().__init__('mission_executor')
        self.waypoints = waypoints
        self.vel_tol = args.vel_tol
        self.settle_sec = args.settle_sec
        self.frame_id = args.frame_id
        self.wait_subscribers = args.wait_subscribers
        self.param_timeout = args.param_timeout
        self.max_attempts = args.param_max_attempts

        self.idx = 0
        self.phase = 'wait_subs'          # wait_subs -> set_params -> fly -> done
        self.close_since = None
        self.have_odom = False
        self.px = self.py = self.pz = 0.0
        self.v = 0.0
        # 参数下发状态
        self.pending_req = None
        self.future = None
        self.attempt = 0
        self.attempt_start = 0.0

        self.pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        odom_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(Odometry, args.odom_topic, self.on_odom, odom_qos)
        param_srv = '/%s/set_parameters' % args.ego_node
        self.cli = self.create_client(SetParameters, param_srv)
        self.ctrl_timer = self.create_timer(0.1, self.control_tick)

        self.get_logger().info(
            'Mission executor: %d wp, odom=%s goal=%s param_srv=%s' % (
                len(waypoints), args.odom_topic, args.goal_topic, param_srv))
        for i, w in enumerate(waypoints):
            p = w['pos']
            self.get_logger().info(
                '  [%d] (%.2f, %.2f, %.2f) %s%s' % (
                    i + 1, p[0], p[1], p[2],
                    'fly_through ' if w['fly_through'] else '',
                    ('params=%s' % w['segment_params']) if w['segment_params'] else ''))

    # ---------- 序列驱动 ----------
    def control_tick(self):
        if self.phase == 'wait_subs':
            n = self.pub.get_subscription_count()
            if n < self.wait_subscribers:
                self.get_logger().info(
                    'Waiting for %d subscribers on goal (have %d)' % (self.wait_subscribers, n),
                    throttle_duration_sec=2.0)
                return
            self.begin_waypoint(0)
        elif self.phase == 'set_params':
            self.poll_params()

    def begin_waypoint(self, i):
        self.idx = i
        wp = self.waypoints[i]
        if wp['segment_params']:
            self.pending_req = self.build_param_request(wp['segment_params'])
            self.attempt = 0
            self.future = None
            self.phase = 'set_params'
            self.issue_param_call()
        else:
            self.publish_goal()
            self.phase = 'fly'
            self.close_since = None

    def advance(self):
        if self.idx >= len(self.waypoints) - 1:
            return
        self.begin_waypoint(self.idx + 1)

    # ---------- 参数下发 ----------
    def build_param_request(self, seg):
        req = SetParameters.Request()
        params = []
        for k, v in seg.items():
            names = _PARAM_MAP.get(k)
            if not names:
                self.get_logger().warn("unknown segment_param '%s', ignored" % k)
                continue
            for name in names:
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
                params.append(Parameter(name=name, value=pv))
        req.parameters = params
        return req

    def issue_param_call(self):
        self.attempt += 1
        self.attempt_start = time.monotonic()
        if not self.cli.service_is_ready():
            self.get_logger().warn(
                'param service not ready (WP %d, attempt %d)' % (self.idx + 1, self.attempt))
            self.future = None
            return
        self.future = self.cli.call_async(self.pending_req)

    def poll_params(self):
        ok = False
        completed = self.future is not None and self.future.done()
        if completed:
            resp = self.future.result()
            ok = resp is not None and len(resp.results) > 0 and all(r.successful for r in resp.results)
        if ok:
            self.get_logger().info('[WP %d] segment params applied' % (self.idx + 1))
            self.finish_params_and_fly()
            return
        # 尚未成功: 判断是否需要重试(已返回被拒 / 服务没起来 / 超时)
        waited = time.monotonic() - self.attempt_start
        need_retry = completed or self.future is None or waited >= self.param_timeout
        if not need_retry:
            return
        if self.attempt >= self.max_attempts:
            self.get_logger().error(
                '[WP %d] param set failed after %d attempts; continuing with previous params'
                % (self.idx + 1, self.max_attempts))
            self.finish_params_and_fly()
            return
        self.issue_param_call()

    def finish_params_and_fly(self):
        self.publish_goal()
        self.phase = 'fly'
        self.close_since = None

    # ---------- 发布 goal ----------
    def publish_goal(self):
        x, y, z = self.waypoints[self.idx]['pos']
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        is_last = (self.idx == len(self.waypoints) - 1)
        self.get_logger().info(
            '[WP %d/%d] publish -> (%.2f, %.2f, %.2f)%s' % (
                self.idx + 1, len(self.waypoints), x, y, z,
                '  (final: px4_offboard takes over)' if is_last else ''))

    # ---------- 到点判定 ----------
    def on_odom(self, msg):
        self.have_odom = True
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pz = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.v = math.sqrt(vx * vx + vy * vy + vz * vz)

        if self.phase != 'fly':
            return
        if self.idx >= len(self.waypoints) - 1:
            return  # 最后一个已发出, 交给 px4_offboard
        if self.reached(self.waypoints[self.idx]):
            self.advance()

    def reached(self, wp):
        x, y, z = wp['pos']
        d_xy = math.hypot(self.px - x, self.py - y)
        if wp['fly_through']:
            # 穿环/引导航点: 只看 XY, 到容差立即切下一点, 不悬停
            if d_xy <= wp['tol_xy']:
                self.get_logger().info(
                    '[WP %d/%d] fly_through reached (d_xy=%.2f)' % (
                        self.idx + 1, len(self.waypoints), d_xy))
                return True
            return False
        # 普通航点: XY+Z+速度小, 且持续 settle 秒
        d_z = abs(self.pz - z)
        now = time.monotonic()
        if d_xy <= wp['tol_xy'] and d_z <= wp['tol_z'] and self.v <= self.vel_tol:
            if self.close_since is None:
                self.close_since = now
                self.get_logger().info(
                    '[WP %d/%d] within tol (d_xy=%.2f d_z=%.2f v=%.2f), settling...' % (
                        self.idx + 1, len(self.waypoints), d_xy, d_z, self.v))
            elif now - self.close_since >= self.settle_sec:
                return True
        else:
            self.close_since = None
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission', required=True, help='path to mission.yaml')
    parser.add_argument('--odom_topic', default='/Odometry')
    parser.add_argument('--goal_topic', default='/move_base_simple/goal')
    parser.add_argument('--frame_id', default='map')
    parser.add_argument('--ego_node', default='drone_0_ego_planner_node')
    parser.add_argument('--vel_tol', type=float, default=0.20)
    parser.add_argument('--settle_sec', type=float, default=1.0)
    parser.add_argument('--wait_subscribers', type=int, default=2)
    parser.add_argument('--param_timeout', type=float, default=1.0)
    parser.add_argument('--param_max_attempts', type=int, default=3)
    # ros2 launch 会附 --ros-args 等, 用 parse_known_args 忽略
    args, _ = parser.parse_known_args()

    waypoints = load_mission(args.mission)
    rclpy.init()
    node = MissionExecutor(waypoints, args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
