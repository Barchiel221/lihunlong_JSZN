// PX4 Offboard 单航点控制器
//
// 状态机:
//   INIT      ─► 等 EKF2 / position 全 valid
//   WARMUP    ─► 50Hz publish 当前位置 setpoint,持续 warmup_seconds 秒
//                目的:满足 PX4 "切 Offboard 前要看到 1~2s setpoint 流" 要求
//   ENGAGE    ─► 发 SET_MODE→Offboard,等 nav_state=14
//   ARMING    ─► 发 ARM_DISARM→arm,等 arming_state=2
//   TAKEOFF   ─► setpoint = (home_x, home_y, -takeoff_alt, home_yaw)
//                到 z 容差范围后转 FLY
//   FLY       ─► setpoint = external trajectory when enabled, yaw follows flight direction
//                (speed deadband + rate limit, seeded from home_yaw_; 改动 I)
//                or (target_x, target_y, -takeoff_alt, target_yaw) in single-waypoint mode
//                xyz+yaw 到容差后转 HOLD
//   FINAL_CORRECTION
//             ─► EGO 轨迹末端到达 RViz goal 后,用 MID360 /Odometry 做最终到点判定
//   HOLD      ─► 在最终目标点悬停 hold_seconds 秒
//   LANDING   ─► 发 NAV_LAND,PX4 自动降落
//   DONE      ─► 等待 disarm
//   ABORT     ─► 任何 failsafe / 超时 / 人工接管 → 立刻 LAND
//
// 安全:
//   - 默认 enable_arm=false,dry-run 模式只 publish setpoint 不解锁
//   - 任何状态超时 → LAND
//   - 检测到 nav_state ≠ OFFBOARD(用户切手动)→ ABORT(交还给飞手)
//   - position invalid / failsafe → LAND
//   - setpoint 在 ROS time 上无重置,timestamp 用 steady_clock

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>

using namespace std::chrono_literals;

namespace {
inline uint64_t now_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}
// 归一化到 (-pi, pi]
inline double wrap_pi(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

class OffboardWaypointNode : public rclcpp::Node {
public:
  OffboardWaypointNode() : Node("px4_offboard_waypoint") {
    // ---- 参数 ----
    declare_parameter<double>("target_x", 0.0);                // NED, m
    declare_parameter<double>("target_y", 0.0);                // NED, m
    declare_parameter<double>("takeoff_altitude", 1.0);        // 正值, NED z = -takeoff_altitude
    declare_parameter<double>("target_yaw_deg", 0.0);          // 0=北, 90=东
    declare_parameter<double>("warmup_seconds", 2.0);
    declare_parameter<double>("hold_seconds", 5.0);
    declare_parameter<double>("reach_xy_tolerance", 0.3);      // m
    declare_parameter<double>("reach_z_tolerance", 0.2);       // m
    declare_parameter<double>("reach_yaw_tolerance_deg", 5.0);
    declare_parameter<double>("max_state_seconds", 30.0);      // 单状态超时
    declare_parameter<double>("setpoint_rate_hz", 50.0);
    // yaw 跟随(改动 I):机头从锁死 home_yaw_ 改为速度死区+限速跟随飞行方向,消除倒飞。
    // yaw_follow_speed 设极大值即退化为锁死 home_yaw_(省赛行为)。
    declare_parameter<double>("yaw_follow_speed", 0.3);        // m/s,水平速度低于此阈值时保持机头
    declare_parameter<double>("yaw_rate_max_deg", 60.0);       // deg/s,机头跟随的最大转向速率
    declare_parameter<bool>("enable_arm", false);              // ★安全默认 false
    declare_parameter<bool>("enable_external_traj", false);    // ★ FLY 阶段是否吃外部 (ego-planner) 轨迹
    declare_parameter<std::string>("external_traj_topic", "/ego/trajectory_setpoint");
    declare_parameter<double>("external_traj_timeout_sec", 0.5);  // 超时回落到 target_x/y hold
    declare_parameter<bool>("enable_final_correction", false);
    declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");
    declare_parameter<std::string>("mid360_odom_topic", "/Odometry");
    declare_parameter<double>("ego_goal_reach_xy_tolerance", 0.35);
    declare_parameter<double>("ego_goal_reach_z_tolerance", 0.25);
    declare_parameter<double>("ego_goal_velocity_tolerance", 0.15);
    declare_parameter<double>("ego_goal_settle_seconds", 1.0);
    declare_parameter<double>("precision_xy_tolerance", 0.12);
    declare_parameter<double>("precision_z_tolerance", 0.15);
    declare_parameter<double>("precision_settle_seconds", 1.0);
    declare_parameter<double>("mid360_odom_timeout_sec", 0.5);
    declare_parameter<bool>("auto_disarm_after_landing", true);
    declare_parameter<double>("land_disarm_delay_sec", 1.0);
    declare_parameter<double>("disarm_retry_seconds", 1.0);

    target_x_      = get_parameter("target_x").as_double();
    target_y_      = get_parameter("target_y").as_double();
    takeoff_alt_   = get_parameter("takeoff_altitude").as_double();
    target_yaw_    = get_parameter("target_yaw_deg").as_double() * M_PI / 180.0;
    warmup_sec_    = get_parameter("warmup_seconds").as_double();
    hold_sec_      = get_parameter("hold_seconds").as_double();
    xy_tol_        = get_parameter("reach_xy_tolerance").as_double();
    z_tol_         = get_parameter("reach_z_tolerance").as_double();
    yaw_tol_       = get_parameter("reach_yaw_tolerance_deg").as_double() * M_PI / 180.0;
    max_state_sec_ = get_parameter("max_state_seconds").as_double();
    enable_arm_    = get_parameter("enable_arm").as_bool();
    enable_ext_traj_ = get_parameter("enable_external_traj").as_bool();
    ext_traj_timeout_ = get_parameter("external_traj_timeout_sec").as_double();
    const auto ext_topic = get_parameter("external_traj_topic").as_string();
    enable_final_correction_ = get_parameter("enable_final_correction").as_bool();
    goal_topic_ = get_parameter("goal_topic").as_string();
    mid360_odom_topic_ = get_parameter("mid360_odom_topic").as_string();
    ego_goal_xy_tol_ = get_parameter("ego_goal_reach_xy_tolerance").as_double();
    ego_goal_z_tol_ = get_parameter("ego_goal_reach_z_tolerance").as_double();
    ego_goal_vel_tol_ = get_parameter("ego_goal_velocity_tolerance").as_double();
    ego_goal_settle_sec_ = get_parameter("ego_goal_settle_seconds").as_double();
    precision_xy_tol_ = get_parameter("precision_xy_tolerance").as_double();
    precision_z_tol_ = get_parameter("precision_z_tolerance").as_double();
    precision_settle_sec_ = get_parameter("precision_settle_seconds").as_double();
    mid360_odom_timeout_ = get_parameter("mid360_odom_timeout_sec").as_double();
    auto_disarm_after_landing_ = get_parameter("auto_disarm_after_landing").as_bool();
    land_disarm_delay_sec_ = get_parameter("land_disarm_delay_sec").as_double();
    disarm_retry_sec_ = get_parameter("disarm_retry_seconds").as_double();
    const double rate_hz = get_parameter("setpoint_rate_hz").as_double();
    rate_hz_          = rate_hz;
    yaw_follow_speed_ = get_parameter("yaw_follow_speed").as_double();
    yaw_rate_max_     = get_parameter("yaw_rate_max_deg").as_double() * M_PI / 180.0;

    // ---- QoS:PX4 DDS 订阅用 best-effort + KeepLast(1) ----
    rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
    px4_qos.best_effort();
    px4_qos.durability_volatile();

    sub_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) { on_pos(m); });
    sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v3", px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr m) { on_status(m); });
    sub_failsafe_ = create_subscription<px4_msgs::msg::FailsafeFlags>(
        "/fmu/out/failsafe_flags", px4_qos,
        [this](const px4_msgs::msg::FailsafeFlags::SharedPtr m) { on_failsafe(m); });
    sub_land_detected_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", px4_qos,
        [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr m) { on_land_detected(m); });

    pub_ocm_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", px4_qos);
    pub_sp_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", px4_qos);
    pub_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", px4_qos);

    if (enable_ext_traj_) {
      // 外部轨迹源用 reliable QoS (ROS2 默认),与 ego_px4_adapter 匹配
      sub_ext_sp_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
          ext_topic, rclcpp::QoS(10),
          [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr m) {
            ext_sp_ = *m;
            ext_sp_recv_time_ = std::chrono::steady_clock::now();
            ext_sp_have_ = true;
          });
      RCLCPP_INFO(get_logger(),
                  "External traj enabled: sub=%s timeout=%.2fs",
                  ext_topic.c_str(), ext_traj_timeout_);
    }
    if (enable_final_correction_) {
      sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic_, rclcpp::QoS(10),
          [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) { on_goal(m); });

      rclcpp::QoS sensor_qos(rclcpp::KeepLast(20));
      sensor_qos.best_effort();
      sub_mid360_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          mid360_odom_topic_, sensor_qos,
          [this](const nav_msgs::msg::Odometry::SharedPtr m) { on_mid360_odom(m); });

      RCLCPP_INFO(get_logger(),
                  "Final correction enabled: goal=%s mid360_odom=%s precision_xy=%.2fm precision_z=%.2fm",
                  goal_topic_.c_str(), mid360_odom_topic_.c_str(),
                  precision_xy_tol_, precision_z_tol_);
    }

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        [this]() { on_timer(); });

    state_start_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
                "Offboard node up. enable_arm=%d  target=(%.2f, %.2f, alt=%.2f m, yaw=%.1f deg)",
                static_cast<int>(enable_arm_), target_x_, target_y_, takeoff_alt_,
                get_parameter("target_yaw_deg").as_double());
    if (!enable_arm_) {
      RCLCPP_WARN(get_logger(),
                  "DRY-RUN: enable_arm=false. Will publish setpoints but NOT engage Offboard / arm.");
    }
  }

private:
  enum class State { INIT, WARMUP, ENGAGE, ARMING, TAKEOFF, FLY, FINAL_CORRECTION, HOLD, LANDING, DONE, ABORT };

  // ============== 回调 ==============
  void on_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
    pos_       = *m;
    pos_valid_ = m->xy_valid && m->z_valid && m->v_xy_valid && m->v_z_valid;
  }
  void on_status(const px4_msgs::msg::VehicleStatus::SharedPtr m) {
    status_         = *m;
    status_recv_ = true;
  }
  void on_failsafe(const px4_msgs::msg::FailsafeFlags::SharedPtr m) {
    failsafe_      = *m;
    failsafe_recv_ = true;
  }
  void on_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr m) {
    land_detected_ = *m;
    land_detected_recv_ = true;
  }
  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr m) {
    goal_map_x_ = m->pose.position.x;
    goal_map_y_ = m->pose.position.y;
    goal_map_z_ = m->pose.position.z;
    goal_have_ = true;
    ego_goal_close_since_ = {};
    precision_close_since_ = {};
    hold_x_ = goal_map_x_;
    hold_y_ = -goal_map_y_;
    hold_z_ = -goal_map_z_;
    hold_yaw_ = home_yaw_;
    RCLCPP_INFO(get_logger(),
                "Captured EGO goal map=(%.2f, %.2f, %.2f), final NED=(%.2f, %.2f, %.2f)",
                goal_map_x_, goal_map_y_, goal_map_z_, hold_x_, hold_y_, hold_z_);
    if (state_ == State::FLY || state_ == State::FINAL_CORRECTION || state_ == State::HOLD) {
      set_state(State::FLY, "new ego goal");
    }
  }
  void on_mid360_odom(const nav_msgs::msg::Odometry::SharedPtr m) {
    mid360_odom_ = *m;
    mid360_odom_recv_ = true;
    mid360_odom_recv_time_ = std::chrono::steady_clock::now();
  }

  // ============== 工具 ==============
  double seconds_in_state() const {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - state_start_).count();
  }
  void set_state(State s, const char* why = "") {
    state_       = s;
    state_start_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "State -> %s  (%s)", state_name(s), why);
  }
  static const char* state_name(State s) {
    switch (s) {
      case State::INIT:    return "INIT";
      case State::WARMUP:  return "WARMUP";
      case State::ENGAGE:  return "ENGAGE";
      case State::ARMING:  return "ARMING";
      case State::TAKEOFF: return "TAKEOFF";
      case State::FLY:     return "FLY";
      case State::FINAL_CORRECTION: return "FINAL_CORRECTION";
      case State::HOLD:    return "HOLD";
      case State::LANDING: return "LANDING";
      case State::DONE:    return "DONE";
      case State::ABORT:   return "ABORT";
    }
    return "?";
  }
  bool reached(double tx, double ty, double tz, double tyaw) const {
    if (!pos_valid_) return false;
    const double ex = tx - pos_.x;
    const double ey = ty - pos_.y;
    const double ez = tz - pos_.z;
    if (std::hypot(ex, ey) > xy_tol_) return false;
    if (std::fabs(ez) > z_tol_)        return false;
    double eyaw = tyaw - pos_.heading;
    while (eyaw > M_PI)  eyaw -= 2 * M_PI;
    while (eyaw < -M_PI) eyaw += 2 * M_PI;
    return std::fabs(eyaw) <= yaw_tol_;
  }
  bool ext_sp_fresh() const {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - ext_sp_recv_time_).count()
           <= ext_traj_timeout_;
  }
  bool mid360_odom_fresh() const {
    return mid360_odom_recv_ &&
           std::chrono::duration<double>(
               std::chrono::steady_clock::now() - mid360_odom_recv_time_).count()
           <= mid360_odom_timeout_;
  }
  static bool finite_value(double v) {
    return std::isfinite(v);
  }
  static double finite_norm3(const std::array<float, 3>& v) {
    if (!finite_value(v[0]) || !finite_value(v[1]) || !finite_value(v[2])) {
      return NAN;
    }
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }
  bool ego_traj_at_goal() {
    if (!enable_final_correction_ || !goal_have_ || !ext_sp_have_ || !ext_sp_fresh()) {
      ego_goal_close_since_ = {};
      return false;
    }

    const double goal_ned_x = goal_map_x_;
    const double goal_ned_y = -goal_map_y_;
    const double goal_ned_z = -goal_map_z_;
    const double ex = static_cast<double>(ext_sp_.position[0]) - goal_ned_x;
    const double ey = static_cast<double>(ext_sp_.position[1]) - goal_ned_y;
    const double ez = static_cast<double>(ext_sp_.position[2]) - goal_ned_z;
    const double v_norm = finite_norm3(ext_sp_.velocity);
    const bool velocity_ok = !std::isfinite(v_norm) || v_norm <= ego_goal_vel_tol_;
    const bool close = std::hypot(ex, ey) <= ego_goal_xy_tol_ &&
                       std::fabs(ez) <= ego_goal_z_tol_ &&
                       velocity_ok;
    if (!close) {
      ego_goal_close_since_ = {};
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (ego_goal_close_since_.time_since_epoch().count() == 0) {
      ego_goal_close_since_ = now;
      return false;
    }
    return std::chrono::duration<double>(now - ego_goal_close_since_).count()
           >= ego_goal_settle_sec_;
  }
  bool mid360_reached_goal() {
    if (!goal_have_ || !mid360_odom_fresh()) {
      precision_close_since_ = {};
      return false;
    }

    const double ex = goal_map_x_ - mid360_odom_.pose.pose.position.x;
    const double ey = goal_map_y_ - mid360_odom_.pose.pose.position.y;
    const double ez = goal_map_z_ - mid360_odom_.pose.pose.position.z;
    const bool close = std::hypot(ex, ey) <= precision_xy_tol_ &&
                       std::fabs(ez) <= precision_z_tol_;
    if (!close) {
      precision_close_since_ = {};
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (precision_close_since_.time_since_epoch().count() == 0) {
      precision_close_since_ = now;
      return false;
    }
    return std::chrono::duration<double>(now - precision_close_since_).count()
           >= precision_settle_sec_;
  }
  bool any_failsafe() const {
    if (!failsafe_recv_) return false;
    return failsafe_.local_position_invalid ||
           failsafe_.local_velocity_invalid ||
           failsafe_.attitude_invalid ||
           failsafe_.battery_low_remaining_time ||
           failsafe_.battery_unhealthy ||
           failsafe_.fd_critical_failure ||
           failsafe_.fd_motor_failure ||
           failsafe_.fd_alt_loss;
  }
  bool user_took_over() const {
    if (!enable_arm_) return false;
    if (!status_recv_) return false;
    // ENGAGE 是"已发 SET_MODE,等 nav_state 翻到 OFFBOARD"的过渡态。
    // PX4 VehicleStatus 发布率 ~1Hz,本地缓存有几百 ms 滞后,这段时间
    // nav_state 一定还是旧值;不能当作用户接管。ENGAGE 自身有 5s 重试 +
    // 全局 max_state_seconds 兜底超时,足够保护。
    if (state_ == State::INIT || state_ == State::WARMUP ||
        state_ == State::ENGAGE) return false;
    return status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
           status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
  }

  // ============== publish 帮手 ==============
  void publish_offboard_control() {
    px4_msgs::msg::OffboardControlMode m{};
    m.timestamp        = now_us();
    m.position         = true;   // 我用位置控制
    m.velocity         = false;
    m.acceleration     = false;
    m.attitude         = false;
    m.body_rate        = false;
    pub_ocm_->publish(m);
  }
  void publish_setpoint(double x, double y, double z, double yaw) {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp   = now_us();
    sp.position[0] = static_cast<float>(x);
    sp.position[1] = static_cast<float>(y);
    sp.position[2] = static_cast<float>(z);
    // velocity/acceleration/jerk 设 NaN = 不控制(只控位置)
    sp.velocity     = {NAN, NAN, NAN};
    sp.acceleration = {NAN, NAN, NAN};
    sp.jerk         = {NAN, NAN, NAN};
    sp.yaw          = static_cast<float>(yaw);
    sp.yawspeed     = NAN;
    pub_sp_->publish(sp);
  }
  void send_command(uint16_t cmd, float p1 = 0.f, float p2 = 0.f,
                    float p3 = 0.f, float p4 = 0.f,
                    float p5 = 0.f, float p6 = 0.f, float p7 = 0.f) {
    px4_msgs::msg::VehicleCommand m{};
    m.timestamp        = now_us();
    m.command          = cmd;
    m.param1           = p1;
    m.param2           = p2;
    m.param3           = p3;
    m.param4           = p4;
    m.param5           = p5;
    m.param6           = p6;
    m.param7           = p7;
    m.target_system    = 1;
    m.target_component = 1;
    m.source_system    = 255;
    m.source_component = 1;
    m.from_external    = true;
    pub_cmd_->publish(m);
  }
  void cmd_set_offboard() {
    // VEHICLE_CMD_DO_SET_MODE: param1=base_mode(1=custom), param2=PX4 main mode(6=offboard)
    send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }
  void cmd_arm() {
    // VEHICLE_CMD_COMPONENT_ARM_DISARM: param1=1.0 arm
    send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
  }
  void cmd_disarm() {
    // VEHICLE_CMD_COMPONENT_ARM_DISARM: param1=0.0 disarm
    send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f);
  }
  void cmd_land() {
    send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }
  void publish_hold_target() {
    publish_setpoint(hold_x_, hold_y_, hold_z_, hold_yaw_);
  }

  // ============== 主时序 ==============
  void on_timer() {
    // 每个 tick 都必须 publish OffboardControlMode + 一个 setpoint(>=2Hz 否则失保)
    publish_offboard_control();

    // 全局安全检查
    if (state_ != State::INIT && state_ != State::WARMUP &&
        state_ != State::DONE && state_ != State::ABORT) {
      if (any_failsafe()) {
        RCLCPP_ERROR(get_logger(), "Failsafe triggered -> LANDING");
        if (enable_arm_) cmd_land();
        set_state(State::LANDING, "failsafe");
      } else if (user_took_over()) {
        RCLCPP_WARN(get_logger(), "User took control (nav_state=%u) -> ABORT", status_.nav_state);
        set_state(State::ABORT, "user override");
      } else if (seconds_in_state() > max_state_sec_ &&
                 !(state_ == State::HOLD && !enable_arm_)) {
        // dry-run(enable_arm_=false) 下 HOLD 是"永久 park"(见 HOLD case 注释),
        // 不受 max_state_seconds 约束; 否则 30s 后被踢到 LANDING->DONE,
        // 使 dry-run 只有 30s 窗口能收 goal 进 FLY(bench 测试无法稳定复现)。
        RCLCPP_ERROR(get_logger(), "State %s timed out (>%.1fs) -> LANDING",
                     state_name(state_), max_state_sec_);
        if (enable_arm_) cmd_land();
        set_state(State::LANDING, "timeout");
      }
    }

    switch (state_) {
      case State::INIT: {
        if (pos_valid_ && status_recv_) {
          // 抓 home(arm 前的当前 NED 位置)
          home_x_   = pos_.x;
          home_y_   = pos_.y;
          home_yaw_ = pos_.heading;
          publish_setpoint(home_x_, home_y_, -takeoff_alt_, home_yaw_);
          set_state(State::WARMUP, "pos valid, capturing home");
        } else {
          // INIT 阶段 setpoint 用 0,只为了让链路活
          publish_setpoint(0, 0, -0.1, 0);
        }
        break;
      }
      case State::WARMUP: {
        publish_setpoint(home_x_, home_y_, -takeoff_alt_, home_yaw_);
        if (seconds_in_state() >= warmup_sec_) {
          if (!enable_arm_) {
            RCLCPP_INFO(get_logger(), "DRY-RUN done. Holding setpoint forever.");
            set_state(State::HOLD, "dry-run park");
          } else {
            cmd_set_offboard();
            set_state(State::ENGAGE, "set Offboard");
          }
        }
        break;
      }
      case State::ENGAGE: {
        publish_setpoint(home_x_, home_y_, -takeoff_alt_, home_yaw_);
        if (status_recv_ &&
            status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          cmd_arm();
          set_state(State::ARMING, "armed cmd sent");
        } else if (seconds_in_state() > 5.0) {
          RCLCPP_WARN(get_logger(), "Mode switch slow, retry");
          cmd_set_offboard();
          state_start_ = std::chrono::steady_clock::now();
        }
        break;
      }
      case State::ARMING: {
        publish_setpoint(home_x_, home_y_, -takeoff_alt_, home_yaw_);
        if (status_recv_ &&
            status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
          set_state(State::TAKEOFF, "armed, climbing");
        } else if (seconds_in_state() > 5.0) {
          RCLCPP_WARN(get_logger(), "Arm slow, retry");
          cmd_arm();
          state_start_ = std::chrono::steady_clock::now();
        }
        break;
      }
      case State::TAKEOFF: {
        publish_setpoint(home_x_, home_y_, -takeoff_alt_, home_yaw_);
        if (reached(home_x_, home_y_, -takeoff_alt_, home_yaw_)) {
          if (!goal_have_) {
            hold_x_ = target_x_;
            hold_y_ = target_y_;
            hold_z_ = -takeoff_alt_;
            hold_yaw_ = target_yaw_;
          }
          cmd_yaw_ = home_yaw_;   // yaw 跟随从真实起飞朝向播种,起飞段落在死区内不 snap
          set_state(State::FLY, "altitude reached");
        }
        break;
      }
      case State::FLY: {
        if (enable_ext_traj_ && ext_sp_have_ && ext_sp_fresh()) {
          // 转发 EGO 轨迹的位置/速度/加速度。yaw 不再锁死 home_yaw_,而是从 EGO 速度方向
          // (NED,atan2(vy,vx) 无需坐标翻转)以"速度死区 + 转向限速"平滑跟随飞行方向:
          //   - 起飞/悬停(speed<yaw_follow_speed_)保持 cmd_yaw_,避免 traj_server 切向 yaw
          //     从初值 0 跳到约 90 deg 引起的起飞猛甩;
          //   - 巡航时机头以 <=yaw_rate_max_ 平滑转向,消除区域②→③右转后的倒飞。
          auto sp = ext_sp_;
          sp.timestamp = now_us();
          const double vx = static_cast<double>(sp.velocity[0]);
          const double vy = static_cast<double>(sp.velocity[1]);
          const double speed = std::hypot(vx, vy);
          double yaw_des = cmd_yaw_;
          if (std::isfinite(speed) && speed > yaw_follow_speed_)
            yaw_des = std::atan2(vy, vx);
          const double max_step = yaw_rate_max_ / rate_hz_;   // dt = 1/rate_hz_
          double dyaw = wrap_pi(yaw_des - cmd_yaw_);
          if (dyaw >  max_step) dyaw =  max_step;
          if (dyaw < -max_step) dyaw = -max_step;
          cmd_yaw_ = wrap_pi(cmd_yaw_ + dyaw);
          sp.yaw       = static_cast<float>(cmd_yaw_);
          sp.yawspeed  = NAN;
          pub_sp_->publish(sp);
          if (ego_traj_at_goal()) {
            hold_x_ = goal_map_x_;
            hold_y_ = -goal_map_y_;
            hold_z_ = -goal_map_z_;
            hold_yaw_ = home_yaw_;
            set_state(State::FINAL_CORRECTION, "ego goal reached, using MID360 odom");
          }
        } else {
          if (enable_ext_traj_ && !goal_have_) {
            publish_setpoint(home_x_, home_y_, -takeoff_alt_, home_yaw_);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Waiting for EGO goal; holding takeoff point");
            break;
          }
          if (enable_ext_traj_ && ext_sp_have_ && !ext_sp_fresh()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "External traj timeout, falling back to target waypoint");
          }
          publish_setpoint(target_x_, target_y_, -takeoff_alt_, target_yaw_);
          if (!enable_ext_traj_ &&
              reached(target_x_, target_y_, -takeoff_alt_, target_yaw_)) {
            set_state(State::HOLD, "waypoint reached");
          }
        }
        break;
      }
      case State::FINAL_CORRECTION: {
        publish_hold_target();
        if (!mid360_odom_fresh()) {
          precision_close_since_ = {};
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Waiting for fresh MID360 odom on %s before final hold",
                               mid360_odom_topic_.c_str());
        } else if (mid360_reached_goal()) {
          set_state(State::HOLD, "MID360 precision goal reached");
        }
        break;
      }
      case State::HOLD: {
        publish_hold_target();
        if (enable_arm_ && seconds_in_state() >= hold_sec_) {
          cmd_land();
          set_state(State::LANDING, "hold done");
        }
        // dry-run 永远停在 HOLD,只发 setpoint,不 land 不 disarm
        break;
      }
      case State::LANDING: {
        // 不再发自定义 setpoint,PX4 LAND 模式接管;但 OffboardControlMode 已经在每 tick publish
        // 仍然 publish 当前位置 setpoint 兜底,避免 PX4 拒绝 LAND
        publish_setpoint(hold_x_, hold_y_, pos_.z, pos_.heading);
        if (enable_arm_ && auto_disarm_after_landing_ &&
            land_detected_recv_ && land_detected_.landed &&
            seconds_in_state() >= land_disarm_delay_sec_ &&
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_disarm_cmd_time_).count()
            >= disarm_retry_sec_) {
          cmd_disarm();
          last_disarm_cmd_time_ = std::chrono::steady_clock::now();
          RCLCPP_WARN(get_logger(), "Landed detected -> disarm command sent");
        }
        if (status_recv_ &&
            status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
          set_state(State::DONE, "disarmed");
        }
        break;
      }
      case State::ABORT: {
        // 用户已接管,不再 publish 任何指令(避免和飞手摇杆打架)
        // 仅保留 OffboardControlMode publish 让 PX4 知道我们还在(防止幽灵 setpoint)
        // 注:实际上 nav_state 已经不是 OFFBOARD,PX4 会忽略我们
        break;
      }
      case State::DONE: {
        // 不动作
        break;
      }
    }
  }

  // ============== 成员 ==============
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        sub_status_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr        sub_failsafe_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr   sub_land_detected_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   sub_ext_sp_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr      sub_goal_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             sub_mid360_odom_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr     pub_ocm_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr      pub_sp_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr          pub_cmd_;
  rclcpp::TimerBase::SharedPtr                                          timer_;

  // 参数
  double target_x_, target_y_, target_yaw_;
  double takeoff_alt_;
  double warmup_sec_, hold_sec_;
  double xy_tol_, z_tol_, yaw_tol_;
  double max_state_sec_;
  bool   enable_arm_;
  bool   enable_ext_traj_ = false;
  double ext_traj_timeout_ = 0.5;
  px4_msgs::msg::TrajectorySetpoint ext_sp_{};
  bool   ext_sp_have_ = false;
  std::chrono::steady_clock::time_point ext_sp_recv_time_;
  bool enable_final_correction_ = false;
  std::string goal_topic_;
  std::string mid360_odom_topic_;
  double ego_goal_xy_tol_ = 0.35;
  double ego_goal_z_tol_ = 0.25;
  double ego_goal_vel_tol_ = 0.15;
  double ego_goal_settle_sec_ = 1.0;
  double precision_xy_tol_ = 0.12;
  double precision_z_tol_ = 0.15;
  double precision_settle_sec_ = 1.0;
  double mid360_odom_timeout_ = 0.5;
  bool auto_disarm_after_landing_ = true;
  double land_disarm_delay_sec_ = 1.0;
  double disarm_retry_sec_ = 1.0;
  double rate_hz_ = 50.0;            // 定时器频率,yaw 限速用其求 dt
  double yaw_follow_speed_ = 0.3;    // yaw 跟随速度死区
  double yaw_rate_max_ = 60.0 * M_PI / 180.0;  // yaw 跟随最大转向速率 (rad/s)

  // 状态
  State state_ = State::INIT;
  std::chrono::steady_clock::time_point state_start_;
  double home_x_ = 0, home_y_ = 0, home_yaw_ = 0;
  double hold_x_ = 0, hold_y_ = 0, hold_z_ = -1.0, hold_yaw_ = 0;
  double cmd_yaw_ = 0;   // yaw 跟随的当前命令值(NED),进入 FLY 时以 home_yaw_ 播种
  bool goal_have_ = false;
  double goal_map_x_ = 0, goal_map_y_ = 0, goal_map_z_ = 0;
  std::chrono::steady_clock::time_point ego_goal_close_since_;
  std::chrono::steady_clock::time_point precision_close_since_;
  std::chrono::steady_clock::time_point last_disarm_cmd_time_;

  // 数据
  px4_msgs::msg::VehicleLocalPosition pos_{};
  bool                                pos_valid_ = false;
  px4_msgs::msg::VehicleStatus        status_{};
  bool                                status_recv_ = false;
  px4_msgs::msg::FailsafeFlags        failsafe_{};
  bool                                failsafe_recv_ = false;
  px4_msgs::msg::VehicleLandDetected  land_detected_{};
  bool                                land_detected_recv_ = false;
  nav_msgs::msg::Odometry             mid360_odom_{};
  bool                                mid360_odom_recv_ = false;
  std::chrono::steady_clock::time_point mid360_odom_recv_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardWaypointNode>());
  rclcpp::shutdown();
  return 0;
}
