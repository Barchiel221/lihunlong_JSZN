// ego_px4_adapter_node
//   in : /position_cmd       (quadrotor_msgs::PositionCommand, map frame)
//   out: /ego/trajectory_setpoint (px4_msgs::TrajectorySetpoint, PX4 local NED)
//
// 由 px4_offboard_pkg 在 FLY 状态下转发到 /fmu/in/trajectory_setpoint，避免双源争抢。
// 当前地图坐标: x 向地图上方/North,y 向地图左方/West,z 向上。
// PX4 NED:      x North,y East,z Down。
// map→NED:      (x,y,z)_NED = (x_map, -y_map, -z_map)；yaw_NED = -yaw_map。
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

class EgoPx4Adapter : public rclcpp::Node {
public:
  EgoPx4Adapter() : Node("ego_px4_adapter") {
    declare_parameter<std::string>("input_topic", "/position_cmd");
    declare_parameter<std::string>("output_topic", "/ego/trajectory_setpoint");
    declare_parameter<bool>("publish_velocity", true);
    declare_parameter<bool>("publish_acceleration", true);

    const auto in_topic  = get_parameter("input_topic").as_string();
    const auto out_topic = get_parameter("output_topic").as_string();
    pub_vel_ = get_parameter("publish_velocity").as_bool();
    pub_acc_ = get_parameter("publish_acceleration").as_bool();

    rclcpp::QoS qos(10);
    pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(out_topic, qos);
    sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        in_topic, qos,
        std::bind(&EgoPx4Adapter::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ego_px4_adapter up: %s -> %s",
                in_topic.c_str(), out_topic.c_str());
  }

private:
  static float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
  }

  void cb(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000ULL);

    // Position: map(x north,y west,z up) -> NED(x north,y east,z down)
    sp.position[0] = static_cast<float>(msg->position.x);
    sp.position[1] = static_cast<float>(-msg->position.y);
    sp.position[2] = static_cast<float>(-msg->position.z);

    if (pub_vel_) {
      sp.velocity[0] = static_cast<float>(msg->velocity.x);
      sp.velocity[1] = static_cast<float>(-msg->velocity.y);
      sp.velocity[2] = static_cast<float>(-msg->velocity.z);
    } else {
      sp.velocity[0] = sp.velocity[1] = sp.velocity[2] = NAN;
    }

    if (pub_acc_) {
      sp.acceleration[0] = static_cast<float>(msg->acceleration.x);
      sp.acceleration[1] = static_cast<float>(-msg->acceleration.y);
      sp.acceleration[2] = static_cast<float>(-msg->acceleration.z);
    } else {
      sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
    }

    sp.jerk[0] = sp.jerk[1] = sp.jerk[2] = NAN;

    // Yaw: map(FLU,X=机头,Y=机头左) -> PX4 NED
    // 两个系只差 Y 反向 + Z 反向,等价于绕 X 轴 180°。yaw 转换:
    //   yaw_ned  = -yaw_map
    //   yawrate_ned = -yawrate_map
    sp.yaw      = wrap_pi(static_cast<float>(-msg->yaw));
    sp.yawspeed = static_cast<float>(-msg->yaw_dot);
    pub_->publish(sp);
  }

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr       pub_;
  bool pub_vel_;
  bool pub_acc_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoPx4Adapter>());
  rclcpp::shutdown();
  return 0;
}
