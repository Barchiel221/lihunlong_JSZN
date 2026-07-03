#ifndef PID_CONTROL_PKG_PID_CONTROL_H
#define PID_CONTROL_PKG_PID_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>

namespace pid_control_pkg {

class PidControl : public rclcpp::Node {
public:
    explicit PidControl(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PidControl() = default;

private:
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string source_frame_ = "map";
    std::string target_frame_ = "laser_link";

    double x_ = 0.0, y_ = 0.0, z_ = 0.0, yaw_ = 0.0;
    double tx_ = 0.0, ty_ = 0.0, tz_ = 0.0, tyaw_ = 0.0;

    double kp_x_ = 1.0, ki_x_ = 0.0, kd_x_ = 0.0;
    double kp_y_ = 1.0, ki_y_ = 0.0, kd_y_ = 0.0;
    double kp_z_ = 1.0, ki_z_ = 0.0, kd_z_ = 0.0;
    double kp_yaw_ = 1.0, ki_yaw_ = 0.0, kd_yaw_ = 0.0;

    double v_max_xy_ = 2.0, v_max_z_ = 1.2, v_max_yaw_ = 1.5;
    double i_max_xy_ = 1.0, i_max_z_ = 0.8, i_max_yaw_ = 1.0;
    double pos_deadzone_ = 0.03, yaw_deadzone_ = 0.02;

    double ix_ = 0.0, iy_ = 0.0, iz_ = 0.0, iyaw_int_ = 0.0;
    double prev_ex_ = 0.0, prev_ey_ = 0.0, prev_ez_ = 0.0, prev_eyaw_ = 0.0;

    rclcpp::Time last_time_;

    void timerCallback();
    void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void updateStateFromTF();
    void computeAndPublish();
};

} // namespace pid_control_pkg

#endif // PID_CONTROL_PKG_PID_CONTROL_H
