#ifndef UART_TO_STM32_H
#define UART_TO_STM32_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int16.hpp>
#include <serial_comm/serial_comm.h>
#include <Eigen/Dense>
#include <memory>

namespace uart_to_stm32 {

class UartToStm32 : public rclcpp::Node {
public:
    explicit UartToStm32(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UartToStm32();

    bool initialize();

private:
    void lookupTransform();
    void processTfTransform(const geometry_msgs::msg::TransformStamped& transform);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    Eigen::Vector3d transformVelocity(const Eigen::Vector3d& linear, double yaw);
    void sendVelocityToSerial(const Eigen::Vector3d& transformed_velocity);
    void protocolDataHandler(uint8_t id, const std::vector<uint8_t>& data);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;
    std::unique_ptr<serial_comm::SerialComm> serial_comm_;

    double update_rate_;
    std::string source_frame_;
    std::string target_frame_;
    double current_yaw_;
    bool yaw_valid_;
    geometry_msgs::msg::Twist current_velocity_;
    bool velocity_valid_;

    static const uint8_t VELOCITY_FRAME_ID = 0x32;
};

} // namespace uart_to_stm32

#endif // UART_TO_STM32_H
