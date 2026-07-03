#include "uart_to_stm32.h"
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace uart_to_stm32 {

UartToStm32::UartToStm32(const rclcpp::NodeOptions& options)
    : rclcpp::Node("uart_to_stm32_node", options),
      current_yaw_(0.0), yaw_valid_(false), velocity_valid_(false)
{
    declare_parameter("update_rate", 100.0);
    declare_parameter("source_frame", std::string("map"));
    declare_parameter("target_frame", std::string("laser_link"));

    RCLCPP_INFO(get_logger(), "UartToStm32 created");
}

UartToStm32::~UartToStm32()
{
    if (timer_) {
        timer_->cancel();
    }
    if (serial_comm_) {
        serial_comm_->stop_protocol_receive();
        serial_comm_->close();
    }
}

bool UartToStm32::initialize()
{
    try {
        update_rate_ = get_parameter("update_rate").as_double();
        source_frame_ = get_parameter("source_frame").as_string();
        target_frame_ = get_parameter("target_frame").as_string();

        RCLCPP_INFO(get_logger(), "UartToStm32 initialized with update rate: %.1f Hz", update_rate_);
        RCLCPP_INFO(get_logger(), "Looking for transform from '%s' to '%s'",
                    source_frame_.c_str(), target_frame_.c_str());

        serial_comm_ = std::make_unique<serial_comm::SerialComm>();
        if (!serial_comm_->initialize("/dev/ttyS6", 921600)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize serial port /dev/ttyS6 at 921600 baudrate: %s",
                         serial_comm_->get_last_error().c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "Serial port /dev/ttyS6 initialized at 921600 baudrate");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / update_rate_));
        timer_ = create_wall_timer(period_ns, [this]() { lookupTransform(); });

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odometryCallback(msg); });

        height_pub_ = create_publisher<std_msgs::msg::Int16>("/height", 10);

        serial_comm_->start_protocol_receive(
            [this](uint8_t id, const std::vector<uint8_t>& data) { protocolDataHandler(id, data); },
            [this](const std::string& err) {
                RCLCPP_WARN(get_logger(), "Serial protocol error: %s", err.c_str());
            });

        RCLCPP_INFO(get_logger(), "UartToStm32 initialized successfully");
        RCLCPP_INFO(get_logger(), "Subscribed to /Odometry topic (LIO measured velocity)");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize: %s", e.what());
        return false;
    }
}

void UartToStm32::lookupTransform()
{
    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero);
        processTfTransform(transform);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(get_logger(), "Transform lookup failed: %s", ex.what());
    }
}

void UartToStm32::processTfTransform(const geometry_msgs::msg::TransformStamped& transform)
{
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;

    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
    yaw_valid_ = true;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Transform %s -> %s: pos(%.3f, %.3f, %.3f) rot(%.3f, %.3f, %.3f)",
        source_frame_.c_str(), target_frame_.c_str(),
        x, y, z, roll, pitch, yaw);

    if (velocity_valid_ && yaw_valid_) {
        Eigen::Vector3d linear_vel(current_velocity_.linear.x,
                                   current_velocity_.linear.y,
                                   current_velocity_.linear.z);
        Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
        sendVelocityToSerial(transformed_vel);
    }
}

void UartToStm32::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_velocity_.linear = msg->twist.twist.linear;
    current_velocity_.angular = msg->twist.twist.angular;
    velocity_valid_ = true;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Measured velocity (map): linear(%.3f, %.3f, %.3f) m/s",
        current_velocity_.linear.x, current_velocity_.linear.y, current_velocity_.linear.z);

    if (yaw_valid_) {
        Eigen::Vector3d linear_vel(current_velocity_.linear.x,
                                   current_velocity_.linear.y,
                                   current_velocity_.linear.z);
        Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
        sendVelocityToSerial(transformed_vel);
    }
}

Eigen::Vector3d UartToStm32::transformVelocity(const Eigen::Vector3d& linear, double yaw)
{
    Eigen::Matrix3d Rz;
    Rz << cos(yaw),  sin(yaw), 0,
          -sin(yaw), cos(yaw), 0,
          0,         0,        1;

    Eigen::Vector3d transformed = Rz * linear;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Velocity transform: yaw=%.3f, original(%.3f,%.3f,%.3f) -> transformed(%.3f,%.3f,%.3f)",
        yaw * 180.0 / M_PI, linear.x(), linear.y(), linear.z(),
        transformed.x(), transformed.y(), transformed.z());

    return transformed;
}

void UartToStm32::sendVelocityToSerial(const Eigen::Vector3d& transformed_velocity)
{
    if (!serial_comm_ || !serial_comm_->is_open()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Serial port is not open, cannot send velocity data");
        return;
    }

    try {
        const double scale_factor = 100.0;
        int16_t vel_x = static_cast<int16_t>(transformed_velocity.x() * scale_factor);
        int16_t vel_y = static_cast<int16_t>(transformed_velocity.y() * scale_factor);
        int16_t vel_z = static_cast<int16_t>(transformed_velocity.z() * scale_factor);

        std::vector<uint8_t> data(6);
        data[0] = static_cast<uint8_t>(vel_x & 0xFF);
        data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
        data[2] = static_cast<uint8_t>(vel_y & 0xFF);
        data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
        data[4] = static_cast<uint8_t>(vel_z & 0xFF);
        data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);

        if (serial_comm_->send_protocol_data(VELOCITY_FRAME_ID, 6, data)) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Sent velocity data: x=%d, y=%d, z=%d (cm/s)", vel_x, vel_y, vel_z);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Failed to send velocity data: %s", serial_comm_->get_last_error().c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in sendVelocityToSerial: %s", e.what());
    }
}

void UartToStm32::protocolDataHandler(uint8_t id, const std::vector<uint8_t>& data)
{
    switch (id) {
        case 0x05: {
            if (data.size() < 2) {
                RCLCPP_WARN(get_logger(), "protocolDataHandler: ID 0x05 data too short");
                break;
            }
            int16_t value = static_cast<int16_t>(
                static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
            std_msgs::msg::Int16 msg;
            msg.data = value;
            height_pub_->publish(msg);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Published /height: %d", value);
            break;
        }
        default:
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000,
                "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
            break;
    }
}

} // namespace uart_to_stm32
