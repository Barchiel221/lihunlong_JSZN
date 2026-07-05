// FAST-LIO /Odometry  ──>  vehicle_visual_odometry (PX4 EKF2 vision input)
//
// 链路:
//   /Odometry  (10 Hz, map: x North, y West, z Up; body=FLU)  →  reset 锚点
//   /livox/imu (200 Hz, body=FLU)            →  strapdown 前向积分 + publish
//   /cloud_effected (10 Hz)                  →  退化检测 → 协方差放大
//
// 输出:
//   /fmu/in/vehicle_visual_odometry  (px4_msgs::VehicleOdometry, NED + body-FRD)
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {

// map → NED: 绕 x 轴 180°。等价于 (x,y,z)_map → (x,-y,-z)_NED。
const Eigen::Quaterniond kQ_MAP2NED(0.0, 1.0, 0.0, 0.0);

// FLU → FRD: 绕 x 轴 180°。等价于 (x,y,z)_FLU → (x,-y,-z)_FRD。
const Eigen::Quaterniond kQ_FLU2FRD(0.0, 1.0, 0.0, 0.0);

// NED 重力(+Z 朝下)
constexpr double kGravity = 9.80665;

inline Eigen::Quaterniond expSO3(const Eigen::Vector3d& w) {
  const double n = w.norm();
  if (n < 1e-9) {
    return Eigen::Quaterniond(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();
  }
  const double half = 0.5 * n;
  const double s = std::sin(half) / n;
  return Eigen::Quaterniond(std::cos(half), s * w.x(), s * w.y(), s * w.z());
}

}  // namespace

class LioPx4Bridge : public rclcpp::Node {
public:
  LioPx4Bridge() : Node("lio_px4_bridge") {
    // 参数
    declare_parameter<std::string>("lio_odom_topic", "/Odometry");
    declare_parameter<std::string>("imu_topic", "/livox/imu");
    declare_parameter<std::string>("effect_cloud_topic", "/cloud_effected");
    declare_parameter<std::string>("px4_vo_topic", "/fmu/in/vehicle_visual_odometry");
    declare_parameter<double>("lio_z_offset_m", 0.0);
    declare_parameter<int>("degraded_threshold", 100);
    declare_parameter<double>("degraded_cov_scale", 100.0);
    declare_parameter<double>("position_var", 0.05);
    declare_parameter<double>("orientation_var", 0.01);
    declare_parameter<double>("velocity_var", 0.10);
    declare_parameter<double>("imu_max_dt", 0.05);  // 单步 dt 上限,丢包保护
    declare_parameter<bool>("publish_on_imu", true);

    lio_odom_topic_      = get_parameter("lio_odom_topic").as_string();
    imu_topic_           = get_parameter("imu_topic").as_string();
    effect_cloud_topic_  = get_parameter("effect_cloud_topic").as_string();
    px4_vo_topic_        = get_parameter("px4_vo_topic").as_string();
    lio_z_offset_m_      = get_parameter("lio_z_offset_m").as_double();
    degraded_threshold_  = get_parameter("degraded_threshold").as_int();
    degraded_cov_scale_  = get_parameter("degraded_cov_scale").as_double();
    pos_var_             = get_parameter("position_var").as_double();
    ori_var_             = get_parameter("orientation_var").as_double();
    vel_var_             = get_parameter("velocity_var").as_double();
    imu_max_dt_          = get_parameter("imu_max_dt").as_double();
    publish_on_imu_      = get_parameter("publish_on_imu").as_bool();

    // QoS:LIO/IMU 高频 best-effort
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(20));
    sensor_qos.best_effort();

    // PX4 uXRCE-DDS 输入侧的 QoS:Best Effort + KeepLast(1) 是 PX4 主线推荐配置
    rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
    px4_qos.best_effort();
    px4_qos.durability_volatile();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        lio_odom_topic_, sensor_qos,
        std::bind(&LioPx4Bridge::on_lio_odom, this, _1));
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, sensor_qos,
        std::bind(&LioPx4Bridge::on_imu, this, _1));
    sub_effect_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        effect_cloud_topic_, sensor_qos,
        std::bind(&LioPx4Bridge::on_effect_cloud, this, _1));

    pub_vo_ = create_publisher<px4_msgs::msg::VehicleOdometry>(px4_vo_topic_, px4_qos);

    health_timer_ = create_wall_timer(2s, [this]() {
      RCLCPP_INFO(get_logger(),
                  "rates: lio=%.1f Hz, imu=%.1f Hz, vo_pub=%.1f Hz | anchor_ready=%d cov_scale=%.1f",
                  lio_hz_.load(), imu_hz_.load(), pub_hz_.load(),
                  static_cast<int>(anchor_ready_.load()), cov_scale_.load());
      lio_hz_  = 0.0;
      imu_hz_  = 0.0;
      pub_hz_  = 0.0;
    });

    RCLCPP_INFO(get_logger(),
                "bridge up. lio=%s imu=%s effect=%s out=%s",
                lio_odom_topic_.c_str(), imu_topic_.c_str(),
                effect_cloud_topic_.c_str(), px4_vo_topic_.c_str());
  }

private:
  void on_lio_odom(const nav_msgs::msg::Odometry::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mtx_);
    lio_hz_ = lio_hz_.load() + 0.5;  // 2s 窗口里每帧累加 0.5 ≈ Hz

    const Eigen::Vector3d p_map(
        m->pose.pose.position.x,
        m->pose.pose.position.y,
        m->pose.pose.position.z + lio_z_offset_m_);
    const Eigen::Quaterniond q_map_flu(
        m->pose.pose.orientation.w, m->pose.pose.orientation.x,
        m->pose.pose.orientation.y, m->pose.pose.orientation.z);

    // 速度种子:对位置做差分(/Odometry.twist 在本地 FAST-LIO 是 0,不可信)
    Eigen::Vector3d v_map_seed = Eigen::Vector3d::Zero();
    const rclcpp::Time t_now = m->header.stamp;
    if (have_last_lio_) {
      const double dt = (t_now - last_lio_t_).seconds();
      if (dt > 1e-3 && dt < 1.0) {
        v_map_seed = (p_map - last_lio_p_map_) / dt;
      }
    }
    last_lio_t_     = t_now;
    last_lio_p_map_ = p_map;
    have_last_lio_  = true;

    // 转 NED/FRD
    anchor_p_ned_   = kQ_MAP2NED * p_map;
    anchor_q_ned_frd_ = kQ_MAP2NED * q_map_flu * kQ_FLU2FRD.conjugate();
    anchor_q_ned_frd_.normalize();
    anchor_v_ned_   = kQ_MAP2NED * v_map_seed;
    anchor_t_       = t_now;
    anchor_ready_   = true;

    if (!publish_on_imu_) {
      publish_vo();
    }
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mtx_);
    imu_hz_ = imu_hz_.load() + 0.5;

    const rclcpp::Time t_now = m->header.stamp;

    // FLU → FRD;Livox IMU acc 输出单位是 g(重力倍数),需要 ×9.80665 转 m/s²
    // (livox_ros_driver2/src/lddc.cpp 直接把 raw acc 复制进 sensor_msgs/Imu 没做单位转换)
    const Eigen::Vector3d acc_frd(
        m->linear_acceleration.x, -m->linear_acceleration.y, -m->linear_acceleration.z);
    const Eigen::Vector3d acc_frd_si = acc_frd * kGravity;  // g → m/s^2
    const Eigen::Vector3d gyr_frd(   // gyro 已经是 rad/s
        m->angular_velocity.x, -m->angular_velocity.y, -m->angular_velocity.z);
    last_gyr_frd_ = gyr_frd;

    if (!anchor_ready_.load()) {
      last_imu_t_     = t_now;
      have_last_imu_  = true;
      return;
    }

    if (!have_last_imu_) {
      last_imu_t_    = t_now;
      have_last_imu_ = true;
      return;
    }

    double dt = (t_now - last_imu_t_).seconds();
    last_imu_t_ = t_now;
    if (dt <= 0.0) return;
    if (dt > imu_max_dt_) dt = imu_max_dt_;  // 丢包保护

    // strapdown 一阶欧拉
    const Eigen::Matrix3d R = anchor_q_ned_frd_.toRotationMatrix();
    const Eigen::Vector3d g_ned(0.0, 0.0, kGravity);   // NED +Z 朝下,g_world = (0,0,+9.81)
    // IMU 测的是 specific force f_b = (F_thrust)/m = a_body_inertial - g_body。
    //   推回去:a_world_inertial = R * f_body + g_world
    //   静止机体水平时:f_FRD ≈ (0,0,-9.81),R=I,a_world = (0,0,-9.81)+(0,0,+9.81) = 0 ✓
    const Eigen::Vector3d a_ned_world = R * acc_frd_si + g_ned;
    anchor_v_ned_ += a_ned_world * dt;
    anchor_p_ned_ += anchor_v_ned_ * dt;
    anchor_q_ned_frd_ = (anchor_q_ned_frd_ * expSO3(gyr_frd * dt)).normalized();
    anchor_t_ = t_now;

    if (publish_on_imu_) publish_vo();
  }

  void on_effect_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr m) {
    const uint32_t n = m->width * m->height;
    cov_scale_ = (static_cast<int>(n) < degraded_threshold_) ? degraded_cov_scale_ : 1.0;
  }

  void publish_vo() {
    px4_msgs::msg::VehicleOdometry vo{};
    const uint64_t now_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    vo.timestamp        = now_us;
    vo.timestamp_sample = static_cast<uint64_t>(anchor_t_.nanoseconds() / 1000);

    vo.pose_frame     = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    vo.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

    vo.position[0] = static_cast<float>(anchor_p_ned_.x());
    vo.position[1] = static_cast<float>(anchor_p_ned_.y());
    vo.position[2] = static_cast<float>(anchor_p_ned_.z());

    vo.q[0] = static_cast<float>(anchor_q_ned_frd_.w());
    vo.q[1] = static_cast<float>(anchor_q_ned_frd_.x());
    vo.q[2] = static_cast<float>(anchor_q_ned_frd_.y());
    vo.q[3] = static_cast<float>(anchor_q_ned_frd_.z());

    // NED 世界系 → body FRD
    const Eigen::Vector3d v_body = anchor_q_ned_frd_.conjugate() * anchor_v_ned_;
    vo.velocity[0] = static_cast<float>(v_body.x());
    vo.velocity[1] = static_cast<float>(v_body.y());
    vo.velocity[2] = static_cast<float>(v_body.z());

    vo.angular_velocity[0] = static_cast<float>(last_gyr_frd_.x());
    vo.angular_velocity[1] = static_cast<float>(last_gyr_frd_.y());
    vo.angular_velocity[2] = static_cast<float>(last_gyr_frd_.z());

    const float scale = static_cast<float>(cov_scale_.load());
    for (int i = 0; i < 3; ++i) {
      vo.position_variance[i]    = static_cast<float>(pos_var_) * scale;
      vo.orientation_variance[i] = static_cast<float>(ori_var_) * scale;
      vo.velocity_variance[i]    = static_cast<float>(vel_var_) * scale;
    }
    vo.reset_counter = reset_counter_;
    vo.quality       = 0;

    pub_vo_->publish(vo);
    pub_hz_ = pub_hz_.load() + 0.5;
  }

  // 参数
  std::string lio_odom_topic_, imu_topic_, effect_cloud_topic_, px4_vo_topic_;
  double lio_z_offset_m_;
  int    degraded_threshold_;
  double degraded_cov_scale_;
  double pos_var_, ori_var_, vel_var_;
  double imu_max_dt_;
  bool   publish_on_imu_;

  // 订阅 / 发布
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_effect_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr  pub_vo_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  // 状态(锚点)
  std::mutex          mtx_;
  std::atomic<bool>   anchor_ready_{false};
  Eigen::Vector3d     anchor_p_ned_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond  anchor_q_ned_frd_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d     anchor_v_ned_{Eigen::Vector3d::Zero()};
  rclcpp::Time        anchor_t_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d     last_gyr_frd_{Eigen::Vector3d::Zero()};

  // LIO 差分速度种子
  bool                have_last_lio_{false};
  rclcpp::Time        last_lio_t_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d     last_lio_p_map_{Eigen::Vector3d::Zero()};

  // IMU dt
  bool                have_last_imu_{false};
  rclcpp::Time        last_imu_t_{0, 0, RCL_ROS_TIME};

  // 健康
  std::atomic<double> cov_scale_{1.0};
  uint8_t             reset_counter_{0};

  // 频率统计
  std::atomic<double> lio_hz_{0.0}, imu_hz_{0.0}, pub_hz_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}
